#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

#-------------------------------------------------------------------------------
# FiCSES python library
# by nyacom (C) 2019.09
#-------------------------------------------------------------------------------

import os, sys
import mmap
import struct
import time
import pdb
import subprocess
import re
from enum import Enum, IntEnum

#----------------------------------------------------------
FICSES_MEM_DEV           = u'/dev/mem'

FICSES_PCI_VENID         = '10ee'       # FICSES PCI Vendor ID (Xilinx)
FICSES_PCI_DEVID         = '8028'       # FICSES PCI Device ID

FICSES_PKT_SIZE          = 32           # FICSES packet size (32B = 256bit)
FICSES_PKT_VAL_SIZE      = 22           # FICSES packet value size (exclude padding)
FICSES_PKT_PLD_SIZE      = 16           # FICSES packet payload size (value size)
FICSES_PKT_EFF_PLD_SIZE  = 0x020000     # FICSES packet effective payload size (256KB / 2)

#FICSES_BA0_ADDR          = 0xf6c0000   # The base address is depends...
#FICSES_BA1_ADDR          = 0xf700000   # The base address is depends...
FICSES_BA0_SIZE          = 4*1024*1024  # BA0 size = 4MB
FICSES_BA1_SIZE          = 4*1024       # BA1 size = 4KB
FICSES_BA1_OFFT0         = 0x00000008

FICSES_TXRX_BUF_SIZE     = 0x040000     # 256KB

FICSES_CH2_TX_BUF_OFFT   = 0x080000
FICSES_CH2_RX_BUF_OFFT   = 0x0c0000
FICSES_CH1_TX_BUF_OFFT   = 0x100000
FICSES_CH1_RX_BUF_OFFT   = 0x140000

FICSES_CH1_TX_START      = 0x04000000   # Ctrl
FICSES_CH2_TX_START      = 0x02000000   # Data
FICSES_CH1_RX_START      = 0x00040000   # Ctrl
FICSES_CH2_RX_START      = 0x00020000   # Data
#----------------------------------------------------------
class SES_ID(IntEnum):
    FIC00            = 0,
    FIC01            = 1,
    FIC02            = 2,
    FIC03            = 3,
    FIC04            = 4,
    FIC05            = 5,
    FIC06            = 6,
    FIC07            = 7,
    FIC08            = 8,
    FIC09            = 9,
    FIC10            = 10,
    FIC11            = 11,
    FIC12            = 12,
    FIC13            = 13,
    FIC14            = 14,
    FIC15            = 15,
    FIC16            = 16,
    FIC17            = 17,
    FIC18            = 18,
    FIC19            = 19,
    FIC20            = 20,
    FIC21            = 21,
    FIC22            = 22,
    FIC23            = 23,
    BCAST            = 0xbb,
    HOST             = 0xff,

class SES_CMD(IntEnum):
    CMD_DUMMY        = 0x0,
    CMD_APSTART      = 0x1,
    CMD_WRITE        = 0x2,
    CMD_READ         = 0x3,
    CMD_APRESET      = 0x4,
    CMD_TBLSET       = 0x5,

class SES_REG(IntEnum):
    REG_STAT         = 0xffff,    # (RW) Status register
    REG_TEST         = 0xfffe,    # (R)  Direct R/W to HLS
    REG_LINKUP       = 0xfffd,    # (R)  Link up
    REG_DIPSW        = 0xfffc,    # (R)  Dipsw
    REG_LED          = 0xfffb,    # (RW) LED
    REG_CHUP         = 0xfffa,    # (R)  Channel up
    REG_TIMER        = 0xfff9,    # (R)  Timer
    REG_SLTNUM       = 0xfff8,    # (RW) Slot number register
    REG_PKT0         = 0xfff7,    # (R)  Packet monitor for 0-3
    REG_PKT1         = 0xfff6,    # (R)  Packet monitor for 4-7
    REG_SLOT         = 0x2000,    # (RW) Slot register
    REG_DDR_WR_START = 0xddd1,    # (W)  ddr_write_start on HLS module
    REG_DDR_RD_START = 0xddd2,    # (W)  ddr_read_start on HLS module
    REG_DDR_ADDR     = 0xddda,    # (RW) ddr address
#----------------------------------------------------------

#----------------------------------------------------------
class FICSES:
    def __init__(self):
        self.fd = None
        self.ba_data = None
        self.ba_ctrl = None

        # Get PCI device ID
        base_addr = self._get_pci_baseaddr(FICSES_PCI_VENID, FICSES_PCI_DEVID)
        self.FICSES_BA0_ADDR = int(base_addr[0], 16)
        self.FICSES_BA1_ADDR = int(base_addr[1], 16)

    def __enter__(self):
        self._ficses_open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self._ficses_close()

    #----------------------------------------------------------
    # Get PCI ID
    #----------------------------------------------------------
    def _get_pci_baseaddr(self, ven_id, dev_id):
        lspci_cmd = 'lspci -d {0}:{1} -v'.format(ven_id, dev_id)
        lspci_run = subprocess.run(lspci_cmd, stdout=subprocess.PIPE, shell=True)
        lspci_out = lspci_run.stdout.decode('utf-8')

        addr = []
        for l in lspci_out.split(u'\n'):
            m = re.search(r'Memory at ([a-f0-9]{8})', l)
            if m:
                addr.append(m.group(1))
        
        if addr == []:
            raise Exception("ERROR: Can not find ficses device!")

        return addr

    #----------------------------------------------------------
    def _ficses_mmap(self, fd, ba_addr, ba_size):
        try:
            ba_offset = ba_addr % mmap.ALLOCATIONGRANULARITY
            m = mmap.mmap(fd, 
                        length=ba_size+ba_offset,
                        flags=mmap.MAP_SHARED,
                        prot=mmap.PROT_READ|mmap.PROT_WRITE,
                        offset=ba_addr-ba_offset)
            return m

        except Exception:
            raise IOError(u'ERROR: mmap failed')

        return None

    #----------------------------------------------------------
    def _ficses_reopen(self):
        pass
        #self._ficses_close()
        #self._ficses_open()

    def _ficses_open(self):
        try:
            self.fd = os.open(FICSES_MEM_DEV, os.O_RDWR)
            self.ba_data = self._ficses_mmap(self.fd, self.FICSES_BA0_ADDR, FICSES_BA0_SIZE)
            self.ba_ctrl = self._ficses_mmap(self.fd, self.FICSES_BA1_ADDR, FICSES_BA1_SIZE)

        except Exception:
            raise IOError(u'ERROR: Device open failed')

        return None

    def _ficses_close(self):
        self.ba_data.close()
        self.ba_ctrl.close()
        os.close(self.fd)

    #----------------------------------------------------------
    def _ficses_pack_pkt(self, cmd, fic_dst, fic_src, reg_addr, data):
        if len(data) > 16:
            raise Exception("ERROR: In _ficses_pack_pkt Too large data!")

        # Pack to 256bit word
        v = (0x1 << 9) | cmd   # valid and cmd
        pkt = struct.pack('<16sHBBH', 
                           data, reg_addr, fic_src, fic_dst, v)

        # All packet should be aligned in 256bit (32B)
        pkt += b'\0' * (FICSES_PKT_SIZE - FICSES_PKT_VAL_SIZE)  # Fill
        return pkt

    def _ficses_strip_pkt(self, data):
        # Strip data
        buf = bytearray()
        p = 0
        while p < len(data):
            b = data[p:p+FICSES_PKT_VAL_SIZE]
            data, reg, fic_src, fic_dst, v = struct.unpack('<16sHBBH', b)
            buf += data
            p += FICSES_PKT_SIZE

        return buf

    #----------------------------------------------------------
    def _ficses_ctrl_reg(self, ba_val, busy_wait=False):
        #self.ba_ctrl.seek(FICSES_BA1_OFFT0, os.SEEK_SET)

        ## Check busy bit
        #v = int.from_bytes(self.ba_ctrl.read(4), 'little')
        #if (v & ba_val):
        #    print(hex(v))
        #    print(hex(self.FICSES_BA1_ADDR))
        #    raise Exception("Busy nyao")

        # Write ctrl resgiter
        self.ba_ctrl.seek(FICSES_BA1_OFFT0, os.SEEK_SET)
        self.ba_ctrl.write(ba_val.to_bytes(4, 'little'))

        self.ba_ctrl.seek(FICSES_BA1_OFFT0, os.SEEK_SET)
        #print(self.ba_ctrl.read(4))

        if busy_wait:
            # Wait until the bit goes off...
            self.ba_ctrl.seek(FICSES_BA1_OFFT0, os.SEEK_SET)
            while (int.from_bytes(self.ba_ctrl.read(4), 'little') & ba_val):
                self.ba_ctrl.seek(FICSES_BA1_OFFT0, os.SEEK_SET)
                time.sleep(0.001)       # wait until busy bit is negated

    ##----------------------------------------------------------
    #def _ficses_ctrl_rxreg(self, ba_ctrl):
    #    ## Kick the rx ctrl register
    #    ba_ctrl.seek(FICSES_BA1_OFFT0, os.SEEK_SET)
    #    ba_ctrl.write(FICSES_CH1_RX_START.to_bytes(4, 'little'))  # Activate TX buf send reg

    #    #ba_ctrl.seek(FICSES_BA1_OFFT0, os.SEEK_SET)
    #    #while (int.from_bytes(ba_ctrl.read(4), 'little') & FICSES_RX_START):
    #    #    ba_ctrl.seek(FICSES_BA1_OFFT0, os.SEEK_SET)
    #    #    time.sleep(0.001)       # wait until busy bit is negated

    ##----------------------------------------------------------
    #def _ficses_ctrl_txreg(self, ba_ctrl):
    #    # ---- Kick the tx ctrl register
    #    ba_ctrl.seek(FICSES_BA1_OFFT0, os.SEEK_SET)
    #    ba_ctrl.write(FICSES_CH1_TX_START.to_bytes(4, 'little'))  # Activate TX buf send reg

    #    # Wait until transfer ficses buffer to fic_dst
    #    ba_ctrl.seek(FICSES_BA1_OFFT0, os.SEEK_SET)
    #    while (int.from_bytes(ba_ctrl.read(4), 'little') & FICSES_CH1_TX_START):
    #        ba_ctrl.seek(FICSES_BA1_OFFT0, os.SEEK_SET)
    #        time.sleep(0.001)       # wait until busy bit is negated

    #----------------------------------------------------------
    def ficses_reg_write(self, fic_tgt, r_addr, data):
        pkt = self._ficses_pack_pkt(SES_CMD.CMD_WRITE,
                                    fic_tgt, SES_ID.HOST, r_addr, data)

        self.ba_data.seek(FICSES_CH1_TX_BUF_OFFT, os.SEEK_SET)     # Always ch1 for ctrl send
        self.ba_data.write(pkt)

        # Fill left 8192-1 rows if need... 
        pkt = self._ficses_pack_pkt(SES_CMD.CMD_DUMMY,
                                     0, 0, 0, b'\x00')
        for _ in range(8192-1):
            self.ba_data.write(pkt)

        self._ficses_ctrl_reg(FICSES_CH1_TX_START, busy_wait=True)    # Kick ctrl reg
        self._ficses_reopen()   # instead of flush()

    #----------------------------------------------------------
    def ficses_reg_read(self, fic_tgt, r_addr):
        pkt = self._ficses_pack_pkt(SES_CMD.CMD_READ,
                                    fic_tgt, SES_ID.HOST, r_addr, b'\x00')

        self.ba_data.seek(FICSES_CH1_TX_BUF_OFFT, os.SEEK_SET)     # Always ch1 for ctrl send
        self.ba_data.write(pkt)
        self._ficses_ctrl_reg(FICSES_CH1_TX_START)    # Kick ctrl reg

        #time.sleep(2)
        self._ficses_ctrl_reg(FICSES_CH1_RX_START)    # Kick ctrl reg

        self.ba_data.seek(FICSES_CH1_RX_BUF_OFFT, os.SEEK_SET)
        rx_buf = self.ba_data.read(FICSES_PKT_VAL_SIZE)

        self._ficses_reopen()   # instead of flush()

        return self._ficses_strip_pkt(rx_buf)

    #----------------------------------------------------------
    def ficses_ddr_write(self, fic_dst, addr, data):
        self.ficses_reg_write(fic_dst, SES_REG.REG_DDR_ADDR, addr.to_bytes(16, 'little'))   # Set DDR addr
        self.ficses_reg_write(fic_dst, SES_REG.REG_DDR_WR_START, b'\x01')  # Set write
        self._ficses_data_send_128k(fic_dst, data)
        self._ficses_reopen()   # instead of flush()

    #----------------------------------------------------------
    def ficses_ddr_read(self, fic_dst, addr):
        # Set DDR address register
        self.ficses_reg_write(fic_dst, SES_REG.REG_DDR_ADDR, addr.to_bytes(16, 'little'))
        self._ficses_ctrl_reg(FICSES_CH2_RX_START)    # Kick ctrl reg
        self.ficses_reg_write(fic_dst, SES_REG.REG_DDR_RD_START, b'\x01')  # Transfer start
        b = self._ficses_data_receive_128k()
        self._ficses_reopen()   # instead of flush()
        return b

    #----------------------------------------------------------
    def ficses_send(self, fic_dst, data):
        self._ficses_data_send_128k(fic_dst, data)
        self._ficses_reopen()   # instead of flush()

    #----------------------------------------------------------
    def ficses_receive(self, fic_dst):
        self._ficses_ctrl_reg(FICSES_CH2_RX_START)    # Kick ctrl reg
        b = self._ficses_data_receive_128k()
        self._ficses_reopen()   # instead of flush()
        return b
  
    #----------------------------------------------------------
    # Receive ficses buffer and read
    #----------------------------------------------------------
    def _ficses_data_receive_128k(self):
        self.ba_data.seek(FICSES_CH2_RX_BUF_OFFT, os.SEEK_SET)
        rx_buf = self.ba_data.read(FICSES_TXRX_BUF_SIZE)

        # Strip data
        buf = bytearray(0)
        p = 0
        while p < len(rx_buf):
            b = rx_buf[p:p+FICSES_PKT_VAL_SIZE]
            data, reg, fic_src, fic_dst, v = struct.unpack('<16sHBBH', b)
            buf += data
            p += FICSES_PKT_SIZE

        return buf

    #----------------------------------------------------------
    # Writing data to ficses buffer and send to dst_id
    #----------------------------------------------------------
    def _ficses_data_send_128k(self, fic_dst, data):
        data_left = len(data)
        data_xfer = 0
        while data_left:
            # Send each 128KB
            page = bytearray()
            if data_left < FICSES_PKT_EFF_PLD_SIZE:
                # if data is less 128KB, padding until 128KB 
                page = data[data_xfer:]
                data_left -= len(page)
                data_xfer += len(page)

                page += b'\0' * (FICSES_PKT_EFF_PLD_SIZE - data_left)

            else:
                page = data[data_xfer:data_xfer+FICSES_PKT_EFF_PLD_SIZE]
                data_left -= FICSES_PKT_EFF_PLD_SIZE 
                data_xfer += FICSES_PKT_EFF_PLD_SIZE

            self.ba_data.seek(FICSES_CH2_TX_BUF_OFFT, os.SEEK_SET)   # Always ch1 for data send

            for i in range(0, FICSES_PKT_EFF_PLD_SIZE, FICSES_PKT_PLD_SIZE):
                # ---- Write to FICSES TX buffer
                pld = page[i:i+FICSES_PKT_PLD_SIZE]

                # ---- Create ficses packet
                pkt = self._ficses_pack_pkt(SES_CMD.CMD_DUMMY,
                                            fic_dst,
                                            SES_ID.HOST, 0, pld)

                self.ba_data.write(pkt)
            
            self._ficses_ctrl_reg(FICSES_CH2_TX_START, busy_wait=True)     # Kick ctrl reg

        return data_xfer

    #----------------------------------------------------------
    def ficses_apstart(self, fic_tgt):
        pkt = self._ficses_pack_pkt(SES_CMD.CMD_APSTART, 
                                    fic_tgt, 0, 0, b'\x01')
        self.ba_data.seek(FICSES_CH1_TX_BUF_OFFT, os.SEEK_SET)     # Always use ch1 for ctrl packet
        self.ba_data.write(pkt)
        self._ficses_ctrl_reg(FICSES_CH1_TX_START)    # Kick ctrl reg

    def ficses_apreset(self, fic_tgt):
        pkt = self._ficses_pack_pkt(SES_CMD.CMD_APRESET, 
                                    fic_tgt, 0, 0, b'\x01')
        self.ba_data.seek(FICSES_CH1_TX_BUF_OFFT, os.SEEK_SET)     # Always use ch1 for ctrl pakcet
        self.ba_data.write(pkt)
        self._ficses_ctrl_reg(FICSES_CH1_TX_START)    # Kick ctrl reg

    #----------------------------------------------------------
    def ficses_tblset(self, fic_dst, tbl):
        # Todo
        pass

#----------------------------------------------------------
if __name__ == '__main__':
    #with FICSES() as m:
    #    # ---- reg read/write test ----
    #    m.ficses_reg_write(SES_ID.FIC00, 0xffff, b'\xde')
    #    b = m.ficses_reg_read(SES_ID.FIC00, 0xffff)
    #    print(b)

    with FICSES() as m:
        # --- DDR write test ----
        m.ficses_apreset(SES_ID.FIC00)
        m.ficses_apstart(SES_ID.FIC00)

        with open("test128k", "rb") as f:
            buf = f.read()
            m.ficses_ddr_write(SES_ID.FIC00, 0x00000000, buf)

        #m.ficses_ddr_write(SES_ID.FIC00, 0x00000000, b'\xcc\xdd\xaa\xaa\xbb\xbb\xbb\xbb')
        #m.ficses_ddr_write(SES_ID.FIC00, 0x00000000, b'\xff\xff\xff\xff\xff\xff\xff\xff')

        # --- DDR read test ----
        #m.ficses_apreset(SES_ID.FIC00)
        #m.ficses_apstart(SES_ID.FIC00)
        b = m.ficses_ddr_read(SES_ID.FIC00, 0x00000000)
        with open("ddr_out.bin", "wb") as f:
            f.write(b)

    #print(len(b))
    #print(b)

    #print(hex(FICSES_BA0_ADDR), hex(FICSES_BA1_ADDR))
    #m.ficses_buf_write(SES_ID.FIC00, b'\xab\xcd\x12\xaa')
    #b = m.ficses_buf_read()
    #print(b)
    #pdb.set_trace()