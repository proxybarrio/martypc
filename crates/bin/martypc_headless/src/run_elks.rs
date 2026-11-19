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
        '&'=>return Some((true, Digit7)),'('=>return Some((true, Digit9)),')'=>return Some((true, Digit0)),
        '$'=>return Some((true, Digit4)),'"'=>return Some((true, Quote)),'\''=>Quote,'<'=>return Some((true, Comma)),'*'=>return Some((true, Digit8)),'_'=>return Some((true, Minus)),
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
    // Interrupt latency meter: ELKS_IRQOFF=<step cycles>. While commands run,
    // measures the longest stretch with IF=0 and the longest stretch in which
    // IRQ 4 (COM1) could not be taken: IF=0, or IRQ 0-4 in service at the PIC
    // (ELKS_IRQOFF_MASK, hex, default 1f: in-service bits that count as blocking).
    let irqoff: u32 = std::env::var("ELKS_IRQOFF").ok().and_then(|v| v.parse().ok()).unwrap_or(0);
    let isr_mask: u8 = std::env::var("ELKS_IRQOFF_MASK").ok().and_then(|v| u8::from_str_radix(v.trim_start_matches("0x"), 16).ok()).unwrap_or(0x1F);
    let thr: u64 = std::env::var("ELKS_IRQOFF_THR").ok().and_then(|v| v.parse().ok()).unwrap_or(2480);
    let (mut if0_t, mut blk_t) = (0u64, 0u64);              // start of current stretch, 0 = none
    let (mut if0_at, mut blk_at) = (0u64, 0u64);            // CS:IP at start of current stretch
    let mut if0_max: (u64, u64, u64) = (0, 0, 0);           // (cycles, start cs:ip, end cs:ip)
    let mut blk_max: (u64, u64, u64) = (0, 0, 0);
    let (mut if0_n, mut blk_n) = (0u64, 0u64);              // stretches longer than thr
    let mut blk_hist: HashMap<u64, (u64, u64)> = HashMap::new(); // start cs:ip -> (count over thr, max)
    let mut cur_where: HashMap<u64, u64> = HashMap::new();  // cs:ip samples in the current IF=0 stretch
    let mut where_hist: HashMap<u64, u64> = HashMap::new(); // cs:ip samples in IF=0 stretches over thr
    let mut isr_at = 0u8;
    let mut cur_bwhere: HashMap<u64, u64> = HashMap::new();
    let mut bwhere_hist: HashMap<u64, u64> = HashMap::new();
    let mut bisr_or = 0u8;
    let mut bios_seen = false;
    let mut len_hist: HashMap<(u64, bool), u64> = HashMap::new(); // (length/128, BIOS code seen) -> count
    loop {
        if irqoff > 0 && state == 2 {
            let end = machine.cpu_cycles() + 20_000;
            while machine.cpu_cycles() < end {
                machine.run(irqoff, &mut ec.borrow_mut());
                let now = machine.cpu_cycles();
                let csip = machine.cpu().flat_ip() as u64 | ((machine.cpu().get_register16(Register16::CS) as u64) << 20);
                let ifl = machine.cpu().get_flags() & 0x0200 != 0;
                let isr = machine.bus().pic().as_ref().map(|p| p.isr()).unwrap_or(0);
                let blocked = !ifl || (isr & isr_mask) != 0;
                if !ifl {
                    if if0_t == 0 { if0_t = now; if0_at = csip; cur_where.clear(); isr_at = isr; }
                    *cur_where.entry(csip >> 4 << 4).or_insert(0) += 1;
                }
                else if if0_t != 0 {
                    let d = now - if0_t;
                    if d > if0_max.0 { if0_max = (d, if0_at, csip); println!("ELKS: irqoff   new IF=0 max {} isr at start {:08b}", d, isr_at); }
                    if d > thr {
                        if0_n += 1;
                        for (k, v) in cur_where.iter() { *where_hist.entry(*k).or_insert(0) += v; }
                    }
                    if0_t = 0;
                }
                if blocked {
                    if blk_t == 0 { blk_t = now; blk_at = csip; cur_bwhere.clear(); bisr_or = 0; bios_seen = false; }
                    if (csip >> 20) >= 0xF000 { bios_seen = true; }
                    *cur_bwhere.entry(csip >> 4 << 4).or_insert(0) += 1;
                    bisr_or |= isr;
                }
                else if blk_t != 0 {
                    let d = now - blk_t;
                    if d >= 1024 { *len_hist.entry((d / 256, bios_seen)).or_insert(0) += 1; }
                    if d > blk_max.0 { blk_max = (d, blk_at, csip); }
                    if d > thr {
                        blk_n += 1;
                        let e = blk_hist.entry(blk_at | ((bisr_or as u64) << 40)).or_insert((0, 0));
                        e.0 += 1; if d > e.1 { e.1 = d; }
                        for (k, v) in cur_bwhere.iter() { *bwhere_hist.entry(*k).or_insert(0) += v; }
                    }
                    blk_t = 0;
                }
            }
        } else if prof_path.is_some() && ((state == 2 && idx >= prof_from) || (!booted && std::env::var("ELKS_PROF_BOOT").is_ok())) {
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
        // ELKS_BOOT_TEXT=<text>: stop at the first screen line ending with it
        // (for example "login:"), reporting the boot time, for boots that
        // do not end at a shell prompt
        if let Ok(t) = std::env::var("ELKS_BOOT_TEXT") {
            if !t.is_empty() && scr.iter().any(|l| l.ends_with(t.as_str())) {
                println!("ELKS: boot to '{}' {} cycles = {:.3} s", t, now - start, (now - start) as f64 / (mhz * 1e6));
                break;
            }
        }
        match state {
            0 => if last == "#" {
                if idx == 0 && !booted {
                    booted = true;
                    println!("ELKS: boot to prompt {} cycles = {:.3} s", now - start, (now - start) as f64 / (mhz * 1e6));
                    for l in &scr { if l.starts_with("Kernel text") { println!("ELKS: {}", l); } }
                }
                if idx >= cmds.len() {
                    if std::env::var("ELKS_SCREEN").is_ok() { for l in &scr { println!("SCR|{}", l); } }
                    break;
                }
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
                    if irqoff > 0 {
                        let f = |v: u64| format!("{:04x}:{:04x}", v >> 20, (v & 0xFFFFF).wrapping_sub((v >> 20) << 4) & 0xFFFF);
                        println!("ELKS: irqoff [{}] IF=0 max {} cycles ({} -> {}), {} over {}; IRQ4 blocked max {} cycles ({} -> {}), {} over {}",
                            cmds[idx], if0_max.0, f(if0_max.1), f(if0_max.2), if0_n, thr,
                            blk_max.0, f(blk_max.1), f(blk_max.2), blk_n, thr);
                        let mut v: Vec<_> = blk_hist.iter().collect();
                        v.sort_by(|a, b| b.1.1.cmp(&a.1.1));
                        for (a, (n, m)) in v.iter().take(8) { println!("ELKS: irqoff   from {} isr {:08b} count {} max {}", f(**a & 0xFFFFFFFFFF), (**a >> 40) as u8, n, m); }
                        let mut w: Vec<_> = bwhere_hist.iter().collect();
                        w.sort_by(|a, b| b.1.cmp(a.1));
                        for (a, n) in w.iter().take(12) { println!("ELKS: irqoff   blocked time at {} samples {}", f(**a), n); }
                        bwhere_hist.clear();
                        let mut h: Vec<_> = len_hist.iter().collect();
                        h.sort();
                        for ((b, bios), n) in h { println!("ELKS: irqoff   blocked {}-{} cycles{}: {}", b * 256, b * 256 + 255, if *bios { " (BIOS code)" } else { "" }, n); }
                        len_hist.clear();
                        let mut w: Vec<_> = where_hist.iter().collect();
                        w.sort_by(|a, b| b.1.cmp(a.1));
                        for (a, n) in w.iter().take(10) { println!("ELKS: irqoff   IF=0 time at {} (16-byte bin) samples {}", f(**a), n); }
                        where_hist.clear();
                        if0_max = (0, 0, 0); blk_max = (0, 0, 0); if0_n = 0; blk_n = 0; blk_hist.clear();
                    }
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
