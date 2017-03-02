; Copyright (c) Nuvoton Technology Corp. All rights reserved.


LR_ROM      0x00000000
{
	ER_STARTUP +0
    {
        startup_nuc505Series.o(RESET, +First)   ; vector table
        *(InRoot$$Sections)                     ; library init
        ; If neither (+ input_section_attr) nor (input_section_pattern) is specified, the default is +RO.
        startup_nuc505Series.o                  ; startup
        system_nuc505Series.o(i.SystemInit)    
	}
    
    ; Relocate vector table in SRAM for fast interrupt handling.
    ER_VECTOR2  0x20000000  EMPTY   0x00000400
    {
    }
    
    ER_RO       +0
    {
		*(+RO)
	}
    
    ER_RW       +0
	{
		*(+RW)
	}
    
    ER_ZI       +0
    {
        *(+ZI)
    }
}
ScatterAssert(LoadLimit(LR_ROM) <= 0x00200000)
ScatterAssert(ImageLimit(ER_ZI) <= 0x20020000)
