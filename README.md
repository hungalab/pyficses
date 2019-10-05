# PyFICSES
A python ficses library by nyacom (C) 2019.08

----

## What is this?

It is a ficses library to access FiCSW boards through FiCSES:

* Read/Write data through FiCSES to FiCSW
* Read/Write data to FiCSW DDR 
* Read/Write control registers on funcNxN through FiCSES

----

## Prerequirement

* The library is working and tested in Python3.6 later, with Python3.5 may not work due to unexpected behavior in mmap module.

* _root_ priviledge is required.

----

## Class

#### class SES_ID(IntEnum):
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
    BCAST            = 0xbb,      # Broadcast
    HOST             = 0xff,      # FiCSES host itself


#### class SES_REG(IntEnum):
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

----

## Methods

### _FICSES.ficses_reg_write(fic_tgt, r_addr, data)_

Write _data_ to funcNxN register through FiCSES. 
_fic_tgt_ is target FiCSW ID (int), the value definied in class _SES_ID_. 
_r_addr_ is target register address of funcNxN module, the value defined in class _SES_REG_. 
_data_ is bytes-like object.

__Note: The function is just only send data to corresponding FiCSW. The HLS part should be initilized and started before call this function. To store data to DDR, you need a HLS logic that receiving data and copy it to DDR on HLS design.__


### _FICSES.ficses_reg_read(fic_tgt, r_addr)_
Read data from register of funcNxN through FiCSES. 
_fic_tgt_ is target FiCSW ID (int), the value definied in class _SES_ID_.
_r_addr_ is target register address of funcNxN module, the value defined in class _SES_REG_. 

__Note: The function is just only receive data from corresponding FiCSW. The HLS part should be initilized and started before call this function. To receive data from DDR, you need a HLS logic that reads out DDR and sends out data to FiCSES on HLS design.__

#### Example
    # ---- reg read/write test ----
    with FICSES as m:
        m.ficses_reg_write(SES_ID.FIC00, 0xffff, b'\xde')
        b = m.ficses_reg_read(SES_ID.FIC00, 0xffff)
        print(b)

----
### _ficses_ddr_write(fic_dst, addr, data)_
Write data from DDR-DRAM of FiCSW.
_fic_dst_ is target FiCSW ID (int), the value definied in class _SES_ID_.
_addr_ is target DDR-DRAM address.
_data_ is bytes-like object.

### _ficses_ddr_read(fic_dst, addr, size)_
Read data from DDR-DRAM of FiCSW and return as bytes-like object.
_fic_dst_ is target FiCSW ID (int), the value definied in class _SES_ID_.
_addr_ is target DDR-DRAM address.
_size_ is read data size

#### Example
    with FICSES() as m:
        # --- DDR write test ----
        m.ficses_apreset(SES_ID.FIC00)
        m.ficses_apstart(SES_ID.FIC00)

        with open("test128k", "rb") as f:
            buf = f.read()
            m.ficses_ddr_write(SES_ID.FIC00, 0x00000000, buf)

        # --- DDR read test ----
        # m.ficses_apreset(SES_ID.FIC00)
        # m.ficses_apstart(SES_ID.FIC00)
        b = m.ficses_ddr_read(SES_ID.FIC00, 0x00000000, 1024*128)
        with open("ddr_out.bin", "wb") as f:
            f.write(b)

----
### _FICSES.ficses_send(fic_dst, data)_
Send out FiCSES data to FiCSW. The behavior like _FICSES.ficses_ddr_write()_ without DDR address register setup.

__Note: The function is just only send data to corresponding FiCSW. The HLS part should be initilized and started before call this function.__

----
### _FICSES.ficses_receive(fic_dst)_
Reads out FiCSW data to FiCSES. The behavior like _FICSES.ficses_ddr_read()_ without DDR address register setup.

__Note: The function is just only reads out data from corresponding FiCSW. The HLS part should be initilized and started before call this function.__

----
### _FICSES.ficses_apstart(fic_tgt)_
Activates start HLS signal _ap_start_

#### Example
    m.ficses_apreset(SES_ID.FIC00)

----
### _FICSES.ficses_apreset(fic_tgt)_
Activates reset HLS signal _ap_reset_

NOTE: please call this function twice...

#### Example
    m.ficses_apreset(SES_ID.FIC00)
    m.ficses_apreset(SES_ID.FIC00)

----

### FAQ
Anything?
