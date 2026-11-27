//! Device Service commands sampled at a guest scheduler boundary.
use super::*;
use crate::json::Json;

impl<S: Soc> Machine<S> {
    pub(super) fn service_state_command(&mut self, w: &WebServer, field: &impl Fn(&str) -> Option<String>) -> bool {
        let Some(op) = field("op") else { return false };
        if !matches!(op.as_str(), "flash-snapshot" | "flash-stop" | "state-snapshot" |
                     "state-get" | "state-set" | "checkpoint" | "controller-report" | "sd-media" | "network-fault") { return false; }
        let Some(id) = field("id").and_then(|value| value.parse::<u32>().ok()) else { return true };
        let args = field("args").unwrap_or_default();
        let args: Vec<_> = args.split_ascii_whitespace().collect();
        self.bus.flush_ticks();
        let value = match (op.as_str(), args.as_slice()) {
            ("flash-snapshot" | "flash-stop", []) => self.service_flash_snapshot().ok().map(|bytes| {
                Json::object([("bytes", Json::counter(bytes as u64)), ("cycle", Json::counter(self.bus.cycles()))])
            }),
            ("network-fault", [target @ ("link" | "radio"), action @ ("up" | "down")]) => {
                let applied = self.bus.wifi_link_mut().is_some_and(|link| link.set_fault(*target == "radio", *action == "up"));
                applied.then(|| self.service_network_state()).flatten()
            }
            ("checkpoint", []) => Some(self.service_checkpoint()),
            ("controller-report", []) => Some(Json::object([
                ("checkpoint", self.service_checkpoint()),
                ("soc", Json::Str(self.bus.report())),
                ("board", Json::Str(self.bus.board_ref().report())),
            ])),
            ("state-snapshot", []) => Some(Json::Obj(self.service_snapshot())),
            ("state-get", [path]) => {
                let nodes = self.service_snapshot();
                nodes.into_iter().find(|(name, _)| name == path).map(|(_, value)| value)
            }
            ("state-get", [path, property]) => {
                let nodes = self.service_snapshot();
                nodes.iter().find(|(name, _)| name == path).and_then(|(_, value)| value.get(property)).cloned()
            }
            ("state-set", [path, "input_drive", level]) => {
                self.service_gpio_drive(path, level).then_some(Json::Null)
            }
            ("state-set", ["/board/imu", "sample", sample @ ..]) if sample.len() == 6 => {
                self.bus.board_command("imu", sample).then_some(Json::Null)
            }
            ("state-set", ["/board/sd", "present", present @ ("true" | "false")]) => {
                self.service_sd_present(*present == "true")
            }
            ("sd-media", [present]) if matches!(*present, "insert" | "eject") => {
                self.service_sd_present(*present == "insert")
            }
            _ => None,
        };
        if let Some(value) = value {
            if op == "flash-stop" { self.max_cycles = 0; }
            else { w.send_text(&format!(r#"{{"t":"service","id":{id},"ok":true,"value":{}}}"#, value.encode())); }
        } else {
            w.send_text(&format!(r#"{{"t":"service","id":{id},"ok":false}}"#));
        }
        true
    }

    fn service_checkpoint(&self) -> Json {
        Json::object([
            ("cycle", Json::counter(self.bus.cycles())),
            ("clock_hz", Json::counter(S::CPU_HZ)),
            ("instructions", Json::counter(self.insns())),
            ("interrupts", Json::counter(self.interrupts)),
            ("exceptions", Json::counter(self.exceptions)),
        ])
    }

    fn service_network_state(&self) -> Option<Json> {
        let link = self.bus.wifi_link()?;
        let (link_up, radio_up) = link.fault_state();
        let net = link.net();
        Some(Json::object([
            ("mode", Json::Str(link.network_mode().into())),
            ("link_up", Json::Bool(link_up)),
            ("radio_up", Json::Bool(radio_up)),
            ("tx_frames", Json::counter(link.tx_frames())),
            ("rx_frames", Json::counter(link.rx_frames())),
            ("rx_dropped", Json::counter(link.rx_dropped())),
            ("dhcp_acks", net.map_or(Json::Null, |net| Json::counter(net.dhcp_acks))),
            ("dns_answers", net.map_or(Json::Null, |net| Json::counter(net.dns_answers))),
            ("ntp_answers", net.map_or(Json::Null, |net| Json::counter(net.ntp_answers))),
        ]))
    }

    fn service_sd_present(&mut self, present: bool) -> Option<Json> {
        if !self.bus.set_sd_present(present) { return None; }
        *self.bus.irq_dirty() = true;
        self.service_sd_state()
    }

    fn service_sd_state(&self) -> Option<Json> {
        let state = self.bus.sd_state()?;
        let mut fields = vec![
            ("present".into(), Json::Bool(state.present)),
            ("blocks".into(), Json::counter(state.blocks)),
            ("commands".into(), Json::counter(state.commands)),
            ("blocks_read".into(), Json::counter(state.blocks_read)),
            ("blocks_written".into(), Json::counter(state.blocks_written)),
        ];
        if let Some(stats) = state.spi_reads {
            fields.extend([
                ("accepted_cmd17".into(), Json::counter(stats.accepted_cmd17)),
                ("accepted_cmd18".into(), Json::counter(stats.accepted_cmd18)),
                ("completed_blocks".into(), Json::counter(stats.completed_blocks)),
                ("reset_generation".into(), Json::counter(stats.reset_generation)),
            ]);
        }
        Some(Json::Obj(fields))
    }

    fn service_snapshot(&self) -> Vec<(String, Json)> {
        let board = self.bus.board_ref();
        let cycle = self.service_checkpoint();
        let mut nodes = vec![
            ("/soc".into(), cycle),
            ("/board".into(), Json::object([
                ("name", Json::Str(board.name().into())),
                ("powered", board.soc_powered().map_or(Json::Null, Json::Bool)),
            ])),
            ("/board/panel".into(), Json::object([
                ("frames", Json::counter(board.display_frames())),
                ("version", Json::counter(board.display_version())),
                ("rotation", Json::Int(i64::from(board.display_rotation()))),
            ])),
        ];
        if let Some(net) = self.service_network_state() { nodes.push(("/soc/wifi".into(), net)); }
        if let Some(sd) = self.service_sd_state() { nodes.push(("/board/sd".into(), sd)); }
        if let Some((leds, updates)) = board.leds() { nodes.push(("/board/leds".into(), led_state(leds, updates))); }
        for (name, leds, updates) in board.led_grids() {
            nodes.push((format!("/board/leds/{name}"), led_state(leds, updates)));
        }
        for (path, value) in board.service_state() { nodes.push((format!("/board/{path}"), value)); }
        for pin in 0..64 {
            let Some(state) = self.bus.gpio_state(pin) else { continue };
            nodes.push((format!("/soc/gpio/{pin}"), Json::object([
                ("level", Json::Bool(self.bus.gpio_input() & 1 << pin != 0)),
                ("output_latch", Json::Bool(state.output)),
                ("output_enable", Json::Bool(state.output_enable)),
                ("pull_up", Json::Bool(state.pull_up)),
                ("pull_down", Json::Bool(state.pull_down)),
            ])));
        }
        nodes
    }

    fn service_gpio_drive(&mut self, path: &str, level: &str) -> bool {
        let Some(pin) = path.strip_prefix("/soc/gpio/").and_then(|pin| pin.parse::<u8>().ok())
            .filter(|pin| *pin < 64 && self.bus.gpio_state(*pin).is_some()) else { return false };
        match level {
            "true" => self.bus.gpio_set_input(pin, true),
            "false" => self.bus.gpio_set_input(pin, false),
            "null" => self.bus.gpio_release_input(pin),
            _ => return false,
        }
        *self.bus.irq_dirty() = true;
        true
    }

    fn service_flash_snapshot(&mut self) -> std::io::Result<usize> {
        use std::io::{Error, ErrorKind, Write};
        let dest = self.host.service_flash_out.as_ref()
            .ok_or_else(|| Error::new(ErrorKind::Unsupported, "Flash output is disabled"))?;
        let name = dest.file_name()
            .ok_or_else(|| Error::new(ErrorKind::InvalidInput, "Flash output needs a file name"))?;
        let mut temp_name = name.to_os_string();
        temp_name.push(format!(".{}.tmp", std::process::id()));
        let temp = dest.with_file_name(temp_name);
        let data = self.bus.flash_contents();
        let mut options = std::fs::OpenOptions::new();
        options.write(true).create_new(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            options.mode(0o600);
        }
        let result = {
            let mut file = options.open(&temp)?;
            file.write_all(data).and_then(|_| file.sync_data())
        };
        if let Err(error) = result.and_then(|_| std::fs::rename(&temp, dest)) {
            let _ = std::fs::remove_file(&temp);
            return Err(error);
        }
        Ok(data.len())
    }

}

fn led_state(leds: &[[u8; 3]], updates: u64) -> Json {
    let rgb = leds.iter().map(|pixel| {
        Json::Arr(pixel.iter().map(|&channel| Json::Int(i64::from(channel))).collect())
    }).collect();
    Json::object([("rgb", Json::Arr(rgb)), ("updates", Json::counter(updates))])
}
