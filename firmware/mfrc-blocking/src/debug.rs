use core::fmt::Write;

use crate::{
    consts::{PCDErrorCode, PICCCommand, PICCType, Uid},
    MFRC522,
};
use embedded_hal::spi::SpiDevice;
use heapless::String;

#[allow(async_fn_in_trait)]
pub trait MFRC522Debug {
    fn debug_dump_card(&mut self, uid: &Uid) -> Result<(), PCDErrorCode>;
    fn debug_dump_card_memory(&mut self, uid: &Uid) -> Result<(), PCDErrorCode>;
    fn debug_dump_card_details(&mut self, uid: &Uid) -> Result<(), PCDErrorCode>;
}

impl<S> MFRC522<S>
where
    S: embedded_hal::spi::SpiDevice,
{
    pub fn test(&mut self) {}
}

impl<S> MFRC522Debug for MFRC522<S>
where
    S: embedded_hal::spi::SpiDevice,
{
    fn debug_dump_card(&mut self, uid: &Uid) -> Result<(), PCDErrorCode> {
        self.debug_dump_card_details(&uid)?;
        self.debug_dump_card_memory(&uid)?;

        Ok(())
    }

    fn debug_dump_card_memory(&mut self, uid: &Uid) -> Result<(), PCDErrorCode> {
        let picc_type = PICCType::from_sak(uid.sak);

        match picc_type {
            PICCType::PiccTypeMifare1K
            | PICCType::PiccTypeMifare4K
            | PICCType::PiccTypeMifareMini => {
                let mifare_key = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
                let res = dump_mifare_classic(self, uid, &mifare_key, picc_type);
                if let Err(e) = res {}
            }
            PICCType::PiccTypeMifareUL => {
                let res = dump_mifare_ultralight(self);
                if let Err(e) = res {}
            }
            PICCType::PiccTypeUnknown | PICCType::PiccTypeNotComplete => {
                return Ok(());
            }
            _ => {}
        }

        self.picc_halta()?;
        Ok(())
    }

    fn debug_dump_card_details(&mut self, uid: &Uid) -> Result<(), PCDErrorCode> {
        let mut dbg_line_buff: String<32> = String::new();
        for i in 0..uid.size {
            _ = dbg_line_buff.write_fmt(format_args!(" {:02X}", uid.uid_bytes[i as usize]));
        }
        Ok(())
    }
}

fn dump_mifare_classic<S: SpiDevice>(
    mfrc522: &mut MFRC522<S>,
    uid: &Uid,
    key: &[u8],
    picc_type: PICCType,
) -> Result<(), PCDErrorCode> {
    let sectors_count = match picc_type {
        PICCType::PiccTypeMifareMini => 5,
        PICCType::PiccTypeMifare1K => 16,
        PICCType::PiccTypeMifare4K => 40,
        _ => unreachable!(),
    };

    for i in (0..sectors_count).rev() {
        let res = dump_mifare_classic_sector(mfrc522, uid, key, i);
        if let Err(e) = res {}
    }

    mfrc522.picc_halta()?;
    mfrc522.pcd_stop_crypto1()?;
    Ok(())
}

fn dump_mifare_classic_sector<S: SpiDevice>(
    mfrc522: &mut MFRC522<S>,
    uid: &Uid,
    key: &[u8],
    sector: u8,
) -> Result<(), PCDErrorCode> {
    let mut groups = [0; 4];
    let mut inverted_error = false;

    let first_block;
    let no_of_blocks;
    if sector < 32 {
        no_of_blocks = 4;
        first_block = sector * no_of_blocks;
    } else if sector < 40 {
        no_of_blocks = 16;
        first_block = 128 + (sector - 32) * no_of_blocks;
    } else {
        return Err(PCDErrorCode::Invalid);
    }

    let mut is_sector_trailer = true;
    let mut buff = [0; 18];

    let mut dbg_line_buff: String<128> = String::new();
    for block_offset in (0..no_of_blocks).rev() {
        dbg_line_buff.clear();

        let block_addr = first_block + block_offset;
        if is_sector_trailer {
            mfrc522.pcd_authenticate(
                PICCCommand::PICC_CMD_MF_AUTH_KEY_A,
                first_block,
                &key,
                &uid,
            )?;

            _ = dbg_line_buff.write_fmt(format_args!("  {sector: >2}    "));
        } else {
            _ = dbg_line_buff.push_str("        ");
        }

        _ = dbg_line_buff.write_fmt(format_args!("{block_addr: >3}   "));

        let mut byte_count = 18;
        mfrc522.mifare_read(block_addr, &mut buff, &mut byte_count)?;

        for i in 0..16 {
            _ = dbg_line_buff.write_fmt(format_args!("{:02X} ", buff[i]));
            if i % 4 == 3 {
                _ = dbg_line_buff.push(' ');
            }
        }

        if is_sector_trailer {
            let c1 = buff[7] >> 4;
            let c2 = buff[8] & 0xF;
            let c3 = buff[8] >> 4;
            let c1_ = buff[6] & 0xF;
            let c2_ = buff[6] >> 4;
            let c3_ = buff[7] & 0xF;

            inverted_error = (c1 != (!c1_ & 0xF)) || (c2 != (!c2_ & 0xF)) || (c3 != (!c3_ & 0xF));
            groups[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
            groups[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
            groups[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
            groups[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);

            is_sector_trailer = false;
        }

        let first_in_group;
        let group;

        if no_of_blocks == 4 {
            group = block_offset;
            first_in_group = true;
        } else {
            group = block_offset / 5;
            first_in_group = (group == 3) || (group != (block_offset + 1) / 5);
        }

        if first_in_group {
            let (g1, g2, g3) = (
                (groups[group as usize] >> 2) & 1,
                (groups[group as usize] >> 1) & 1,
                (groups[group as usize] >> 0) & 1,
            );

            _ = core::fmt::write(&mut dbg_line_buff, format_args!("[ {g1} {g2} {g3} ]"));
            if inverted_error {
                _ = dbg_line_buff.push_str(" Inverted access bits did not match! ");
            }
        }

        if group != 3 && (groups[group as usize] == 1 || groups[group as usize] == 6) {
            let val = ((buff[3] as i32) << 24)
                | ((buff[2] as i32) << 16)
                | ((buff[1] as i32) << 8)
                | (buff[0] as i32);

            _ = dbg_line_buff.write_fmt(format_args!(" Value=0x{val:X} Adr=0x{:X}", buff[12]));
        }
    }

    Ok(())
}

fn dump_mifare_ultralight<S: SpiDevice>(mfrc522: &mut MFRC522<S>) -> Result<(), PCDErrorCode> {
    let mut buff = [0; 18];
    let mut i;

    let mut dbg_line_buff: String<128> = String::new();
    for page in (0..16).step_by(4) {
        dbg_line_buff.clear();

        let mut bytes_count = 18;
        mfrc522.mifare_read(page, &mut buff, &mut bytes_count)?;

        for offset in 0..4 {
            i = page + offset;
            _ = dbg_line_buff.write_fmt(format_args!(" {i: >2}  "));

            for index in 0..4 {
                i = 4 * offset + index;
                _ = dbg_line_buff.write_fmt(format_args!(" {i:02X}  "));
            }
        }
    }

    Ok(())
}
