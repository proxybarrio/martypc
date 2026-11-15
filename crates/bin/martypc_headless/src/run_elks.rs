//! ELKS benchmark runner: boots a floppy, types commands at the shell prompt,
//! and reports exact CPU cycles per command (cycle-accurate 8088 timing).
use std::{cell::RefCell, rc::Rc};
use marty_common::types::keys::MartyKey;
use marty_core::machine::{ExecutionControl, Machine};
use marty_core::cpu_common::{Cpu, Register16};
use marty_core::device_traits::videocard::VideoCard;
use std::collections::HashMap;

fn key_for(c: char) -> Option<(bool, MartyKey)> {
    use MartyKey::*;
    let k = match c.to_ascii_lowercase() {
        'a'=>KeyA,'b'=>KeyB,'c'=>KeyC,'d'=>KeyD,'e'=>KeyE,'f'=>KeyF,'g'=>KeyG,'h'=>KeyH,'i'=>KeyI,
        'j'=>KeyJ,'k'=>KeyK,'l'=>KeyL,'m'=>KeyM,'n'=>KeyN,'o'=>KeyO,'p'=>KeyP,'q'=>KeyQ,'r'=>KeyR,
        's'=>KeyS,'t'=>KeyT,'u'=>KeyU,'v'=>KeyV,'w'=>KeyW,'x'=>KeyX,'y'=>KeyY,'z'=>KeyZ,
        '0'=>Digit0,'1'=>Digit1,'2'=>Digit2,'3'=>Digit3,'4'=>Digit4,'5'=>Digit5,'6'=>Digit6,
        '7'=>Digit7,'8'=>Digit8,'9'=>Digit9,' '=>Space,'/'=>Slash,'-'=>Minus,'.'=>Period,
        '='=>Equal,';'=>Semicolon,',' =>Comma,
        '>'=>return Some((true, Period)),'|'=>return Some((true, Backslash)),
        '&'=>return Some((true, Digit7)),'*'=>return Some((true, Digit8)),'_'=>return Some((true, Minus)),
        _=>return Option::None };
    Some((c.is_ascii_uppercase(), k))
}

fn screen(m: &Machine) -> Vec<String> {
    let mut lines = Vec::new();
    // honour the CRTC display start address (hardware scrolling / page flipping)
    let start = m.bus().primary_video().map(|v| v.start_address() as usize).unwrap_or(0);
    for row in 0..25 {
        let mut s = String::new();
        for col in 0..80 {
            let b = m.bus().peek_u8(0xB8000 + (((start + row * 80 + col) * 2) & 0x3FFF)).unwrap_or(b' ');
            s.push(if (0x20..0x7f).contains(&b) { b as char } else { ' ' });
        }
        lines.push(s.trim_end().to_string());
    }
    lines
}
fn last_line(l: &[String]) -> String { l.iter().rev().find(|s| !s.is_empty()).cloned().unwrap_or_default() }

pub fn run_elks(machine: &mut Machine, ec: &Rc<RefCell<ExecutionControl>>, fd: &str) {
    let img = std::fs::read(fd).expect("read floppy image");
    if let Some(fdc) = machine.fdc() { fdc.load_image_from(0, img, None, false).expect("load floppy"); }
    let cmds: Vec<String> = std::env::var("ELKS_CMDS").unwrap_or_default()
        .split("|||").filter(|s| !s.is_empty()).map(|s| s.to_string()).collect();
    let maxsec: f64 = std::env::var("ELKS_MAXSEC").ok().and_then(|v| v.parse().ok()).unwrap_or(900.0);
    let mhz = machine.get_cpu_mhz();
    let max_cycles = (maxsec * mhz * 1e6) as u64;
    let start = machine.cpu_cycles();
    // states: 0 = wait prompt, 1 = typed, waiting for echo, 2 = running, waiting for prompt
    let (mut state, mut idx, mut t0) = (0, 0usize, 0u64);
    let mut seen_busy = false;
    let mut booted = false;
    let mut marks: Vec<(u16, u64)> = std::env::var("ELKS_MARKS").unwrap_or_default().split(',').filter(|x| !x.is_empty()).map(|x| (u16::from_str_radix(x, 16).unwrap_or(0), 0u64)).collect();
    // Cycle-weighted sampling profiler: ELKS_PROF=<file>, samples CS:IP while commands run.
    let prof_path = std::env::var("ELKS_PROF").ok();
    let mut hist: HashMap<u64, u64> = HashMap::new();
    let sample: u32 = std::env::var("ELKS_SAMPLE").ok().and_then(|v| v.parse().ok()).unwrap_or(97);
    let prof_from: usize = std::env::var("ELKS_PROF_FROM").ok().and_then(|v| v.parse().ok()).unwrap_or(0);
    println!("ELKS: cpu {:.4} MHz", mhz);
    loop {
        if prof_path.is_some() && ((state == 2 && idx >= prof_from) || (!booted && std::env::var("ELKS_PROF_BOOT").is_ok())) {
            for _ in 0..206 {
                machine.run(sample, &mut ec.borrow_mut());
                let cs = machine.cpu().get_register16(Register16::CS);
                *hist.entry(machine.cpu().flat_ip() as u64 | ((cs as u64) << 20)).or_insert(0) += 1;
                if !booted {
                    for (k, m) in marks.iter_mut() {
                        if *m == 0 && cs == *k { *m = machine.cpu_cycles() - start; println!("ELKS: mark cs {:x} at {:.3} s", k, *m as f64 / (mhz * 1e6)); }
                    }
                }
            }
        } else {
            machine.run(20_000, &mut ec.borrow_mut());
        }
        let now = machine.cpu_cycles();
        if now - start > max_cycles {
            println!("ELKS: timeout state {} idx {}", state, idx);
            if let Ok(spec) = std::env::var("ELKS_DUMP") {
                for item in spec.split(',') {
                    let v: Vec<usize> = item.split(':').map(|x| usize::from_str_radix(x, 16).unwrap_or(0)).collect();
                    let base = v[0] * 16 + v[1];
                    let bytes: Vec<String> = (0..v[2]).map(|i| format!("{:02x}", machine.bus().peek_u8(base + i).unwrap_or(0))).collect();
                    println!("ELKS: dump {}: {}", item, bytes.join(" "));
                }
            }
            break;
        }
        let scr = screen(machine); let last = last_line(&scr);
        match state {
            0 => if last == "#" {
                if idx == 0 && !booted {
                    booted = true;
                    println!("ELKS: boot to prompt {} cycles = {:.3} s", now - start, (now - start) as f64 / (mhz * 1e6));
                    for l in &scr { if l.starts_with("Kernel text") { println!("ELKS: {}", l); } }
                }
                if idx >= cmds.len() { break; }
                if let Some(secs) = cmds[idx].strip_prefix("@idle ") {
                    // sit at the prompt, sampling, for the given virtual seconds
                    let secs: f64 = secs.trim().parse().unwrap_or(5.0);
                    let end = machine.cpu_cycles() + (secs * mhz * 1e6) as u64;
                    while machine.cpu_cycles() < end {
                        machine.run(sample, &mut ec.borrow_mut());
                        if prof_path.is_some() && idx >= prof_from {
                            *hist.entry(machine.cpu().flat_ip() as u64 | ((machine.cpu().get_register16(Register16::CS) as u64) << 20)).or_insert(0) += 1;
                        }
                    }
                    println!("ELKS: idle {} s done", secs);
                    idx += 1;
                    continue;
                }
                for c in cmds[idx].chars() {
                    if let Some((shift, k)) = key_for(c) {
                        if shift { machine.emit_key_sequence(&[MartyKey::ShiftLeft, k]); } else { machine.emit_key_sequence(&[k]); }
                    }
                }
                state = 1;
            },
            1 => if last == format!("# {}", cmds[idx]) {
                machine.emit_key_sequence(&[MartyKey::Enter]);
                println!("ELKS: enter sent at {}", now - start);
                t0 = now; state = 2; seen_busy = false;
            },
            _ => {
                if last != "#" { seen_busy = true; }
                if last == "#" && seen_busy {
                    let c = now - t0;
                    println!("ELKS: cmd [{}] {} cycles = {:.3} s", cmds[idx], c, c as f64 / (mhz * 1e6));
                    idx += 1; state = 0;
                }
            }
        }
    }
    if let Some(p) = prof_path {
        let mut out = String::new();
        for (a, c) in &hist { out.push_str(&format!("{:x} {:x} {}\n", a & 0xFFFFF, a >> 20, c)); }
        std::fs::write(&p, out).expect("write profile");
        println!("ELKS: profile samples {} written to {}", hist.values().sum::<u64>(), p);
    }
    {
        let start = machine.bus().primary_video().map(|v| v.start_address()).unwrap_or(0);
        let bda = machine.bus().peek_u8(0x44E).unwrap_or(0) as u16 | ((machine.bus().peek_u8(0x44F).unwrap_or(0) as u16) << 8);
        println!("ELKS: crtc start {} words, BIOS 0x44E {} bytes ({})", start, bda,
            if bda == start * 2 { "match" } else { "MISMATCH" });
    }
    println!("ELKS: ---- screen ----");
    for l in screen(machine) { println!("{}", l); }
}
