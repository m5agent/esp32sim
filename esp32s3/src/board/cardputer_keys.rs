//! Key labels from M5Cardputer Keyboard.h.
pub(super) fn position(name: &str) -> Option<(u8, u8)> {
    const ROWS: [[&str; 14]; 4] = [
        ["`", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "-", "=", "del"],
        ["tab", "q", "w", "e", "r", "t", "y", "u", "i", "o", "p", "[", "]", "\\"],
        ["fn", "shift", "a", "s", "d", "f", "g", "h", "j", "k", "l", ";", "'", "enter"],
        ["ctrl", "opt", "alt", "z", "x", "c", "v", "b", "n", "m", ",", ".", "/", "space"],
    ];
    let name = match name { "backspace" | "delete" => "del", "return" => "enter", _ => name };
    ROWS.iter().enumerate().find_map(|(y, keys)| keys.iter().position(|key| key.eq_ignore_ascii_case(name))
        .map(|x| (y as u8, x as u8)))
}
