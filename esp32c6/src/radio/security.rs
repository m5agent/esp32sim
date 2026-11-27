//! IEEE 802.15.4 CCM* transmit offload, using the existing AES primitive.
use esp_periph::crypto::aes128_encrypt_block;

/// Apply the security level in the MAC auxiliary header. The frame already
/// has its MIC space reserved by the guest and does not include the PHY FCS.
pub(super) fn protect(
    psdu: &mut [u8], security: usize, payload: usize, addr: [u8; 8], key: [u8; 16],
) -> Option<()> {
    let control = *psdu.get(security)?;
    let level = control & 7;
    if level == 0 { return Some(()); }
    // The C6 offload reads the legacy explicit 32-bit frame counter.
    let counter: [u8; 4] = psdu.get(security + 1..security + 5)?.try_into().ok()?;
    if control & 0x60 != 0 || payload < security + 5 { return None; }
    let mut nonce = [0; 13];
    for (byte, source) in nonce[..8].iter_mut().zip(addr.into_iter().rev()) { *byte = source; }
    nonce[8..12].copy_from_slice(&u32::from_le_bytes(counter).to_be_bytes());
    nonce[12] = level;

    let mic_len = [0, 4, 8, 16][usize::from(level & 3)];
    let (frame, mic) = psdu.split_at_mut(psdu.len().checked_sub(mic_len)?);
    let encrypt = level & 4 != 0;
    if payload > frame.len() { return None; }
    let (aad_len, payload_len) = if encrypt { (payload, frame.len() - payload) } else { (frame.len(), 0) };

    let mut ctr = [0; 16];
    ctr[0] = 1; // Two bytes for the block counter.
    ctr[1..14].copy_from_slice(&nonce);
    if mic_len != 0 {
        let mut b0 = ctr;
        b0[0] |= ((mic_len as u8 - 2) / 2) << 3;
        b0[0] |= u8::from(aad_len != 0) << 6;
        b0[14..].copy_from_slice(&(payload_len as u16).to_be_bytes());
        let mut tag = aes128_encrypt_block(&key, &b0);
        // A short AAD length has a 16-bit prefix. A PHY frame is at most 127 B.
        let mut aad = [0; 128];
        aad[..2].copy_from_slice(&(aad_len as u16).to_be_bytes());
        aad.get_mut(2..aad_len + 2)?.copy_from_slice(&frame[..aad_len]);
        for block in aad[..aad_len + 2].chunks(16).chain(frame[aad_len..].chunks(16)) {
            for (a, b) in tag.iter_mut().zip(block) { *a ^= b; }
            tag = aes128_encrypt_block(&key, &tag);
        }
        let mask = aes128_encrypt_block(&key, &ctr);
        for (i, b) in mic.iter_mut().enumerate() { *b = tag[i] ^ mask[i]; }
    }
    if encrypt {
        for (i, block) in frame[payload..].chunks_mut(16).enumerate() {
            ctr[14..].copy_from_slice(&((i + 1) as u16).to_be_bytes());
            let mask = aes128_encrypt_block(&key, &ctr);
            for (a, b) in block.iter_mut().zip(mask) { *a ^= b; }
        }
    }
    Some(())
}
