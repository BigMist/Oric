--
-- A simulation model of ORIC ATMOS hardware
-- Copyright (c) SEILEBOST - March 2006
-- 
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- Redistributions of source code must retain the above copyright notice,
-- this list of conditions and the following disclaimer.
--
-- Redistributions in synthesized form must reproduce the above copyright
-- notice, this list of conditions and the following disclaimer in the
-- documentation and/or other materials provided with the distribution.
--
-- Neither the name of the author nor the names of other contributors may
-- be used to endorse or promote products derived from this software without
-- specific prior written permission.
--
-- THIS CODE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- You are responsible for any legal issues arising from your use of this code.
--
-- The latest version of this file can be found at: passionoric.free.fr
--
-- Email seilebost@free.fr
--
--

  library ieee;
  use ieee.std_logic_1164.all;
  use ieee.std_logic_arith.all;
  use ieee.std_logic_unsigned.all;

  
entity oricatmos is
  port (
    CLK_IN            : in    std_logic;
    RESET             : in    std_logic;
	 cpu_type          : in    std_logic_vector(1 downto 0);
	 key_pressed       : in    std_logic;
	 key_extended      : in    std_logic;
	 key_code          : in    std_logic_vector(7 downto 0);
	 key_strobe        : in    std_logic;
    K7_TAPEIN         : in    std_logic;
    K7_TAPEOUT        : out   std_logic;
    K7_REMOTE         : out   std_logic;
	 PSG_OUT           : out   unsigned(13 downto 0);
    PSG_OUT_A         : out   unsigned(11 downto 0);
    PSG_OUT_B         : out   unsigned(11 downto 0);
    PSG_OUT_C         : out   unsigned(11 downto 0);
	 STEREO            : in    std_logic;
    VIDEO_R           : out   std_logic;
    VIDEO_G           : out   std_logic;
    VIDEO_B           : out   std_logic;
    VIDEO_HSYNC       : out   std_logic;
    VIDEO_VSYNC       : out   std_logic;
    VIDEO_HBLANK      : out   std_logic;
    VIDEO_VBLANK      : out   std_logic;
    VIDEO_SYNC        : out   std_logic;
	 BLANKINGn         : out   std_logic;
	 ram_ad            : out std_logic_vector(15 downto 0);
	 ram_d             : out std_logic_vector( 7 downto 0);
	 ram_q             : in  std_logic_vector( 7 downto 0);
	 ram_cs            : out std_logic;
	 ram_oe            : out std_logic;
	 ram_we            : out std_logic;
	 rom_ad            : out std_logic_vector(15 downto 0);
	 rom_q             : in  std_logic_vector( 7 downto 0);
	 rom_cs            : out std_logic;
	 rom_ext_cs        : out std_logic;
	 phi2              : out std_logic;
	 fd_led            : out std_logic;
	 fdd_ready         : in std_logic;
	 fdd_layout        : in std_logic;
	 joystick_0        : in std_logic_vector( 7 downto 0);
	 joystick_1        : in std_logic_vector( 7 downto 0);
	 pll_locked        : in std_logic;
	 disk_enable       : in std_logic;
	 rom               : in std_logic;
	 img_mounted:     in std_logic;
	 img_wp:          in std_logic;
	 img_size:        in std_logic_vector (31 downto 0);
	 sd_lba:          out std_logic_vector (31 downto 0);
	 sd_rd:           out std_logic;
	 sd_wr:           out std_logic;
	 sd_ack:          in std_logic;
	 sd_buff_addr:    in std_logic_vector (8 downto 0);
	 sd_dout:         in std_logic_vector (7 downto 0);
	 sd_din:          out std_logic_vector (7 downto 0);
	 sd_dout_strobe:  in std_logic;
	 sd_din_strobe:   in std_logic
	 );
end;

architecture RTL of oricatmos is
  
    -- Gestion des resets
	 signal RESETn        		: std_logic;
    signal reset_dll_h        : std_logic;
    signal delay_count        : std_logic_vector(7 downto 0) := (others => '0');
    signal clk_cnt            : std_logic_vector(2 downto 0) := "000";

    -- cpu
    signal cpu_di             : std_logic_vector(7 downto 0);
    signal cpu_di_last        : std_logic_vector(7 downto 0);
    signal cpu_ad             : std_logic_vector(23 downto 0);
    signal cpu_do             : std_logic_vector(7 downto 0);
    signal cpu_rw             : std_logic;
	 signal cpu1_ad            : std_logic_vector(23 downto 0);
    signal cpu1_rw            : std_logic;
    signal cpu1_do            : std_logic_vector(7 downto 0);
    signal cpu2_ad            : std_logic_vector(23 downto 0);
    signal cpu2_do            : std_logic_vector(7 downto 0);
    signal cpu2_rw            : std_logic;
    signal cpu_irq            : std_logic;
      
	 -- VIA
    signal via_pa_out_oe_l    : std_logic_vector( 7 downto 0);
    signal via_pa_out_oe      : std_logic_vector( 7 downto 0);
    signal via_pa_in          : std_logic_vector( 7 downto 0);
    signal via_pa_out         : std_logic_vector( 7 downto 0);
    signal via_cb1_out        : std_logic;
    signal via_cb1_oe_l       : std_logic;
    signal via_ca2_out        : std_logic;
    signal via_cb2_out        : std_logic;
    signal via_pb_in             : std_logic_vector( 7 downto 0);
    signal via_pb_out            : std_logic_vector( 7 downto 0);
    signal via_pb_oe_l           : std_logic_vector( 7 downto 0);
    signal VIA_DO             : std_logic_vector( 7 downto 0);
    signal via_irq            : std_logic;

    
    -- Clavier : ÃÂ©mulation par port PS2
    signal KEY_HIT            : std_logic;
    signal KEYB_RESETn        : std_logic;
    signal KEYB_NMIn          : std_logic;

    -- PSG
    signal ym_ioa_out          : std_logic_vector (7 downto 0);
    signal psg_do             : std_logic_vector (7 downto 0);

    -- ULA    
    signal ula_phi2           : std_logic;
    signal ula_CSIOn          : std_logic;
    signal ula_CSROMn         : std_logic;
	 signal ula_CSRAMn         : std_logic;
    signal ula_AD_SRAM        : std_logic_vector(15 downto 0);
    signal ula_CE_SRAM        : std_logic;
    signal ula_OE_SRAM        : std_logic;
    signal ula_WE_SRAM        : std_logic;
	 signal ula_LATCH_SRAM     : std_logic;
    signal ula_MUX            : std_logic;
    signal ula_RW_RAM         : std_logic;
	 signal ula_VIDEO_R        : std_logic;
	 signal ula_VIDEO_G        : std_logic;
	 signal ula_VIDEO_B        : std_logic;
	 

--	 signal lSRAM_D            : std_logic_vector(7 downto 0);
	 signal ENA_1MHZ           : std_logic;
	 signal ENA_1MHZ_N         : std_logic;
    signal ROM_ATMOS_DO     	: std_logic_vector(7 downto 0);
	 signal ROM_1_DO     	   : std_logic_vector(7 downto 0);
	 signal ROM_MD_DO          : std_logic_vector(7 downto 0);
	 
	 --- Printer port
	 signal PRN_STROBE			: std_logic;
	 signal PRN_DATA           : std_logic_vector(7 downto 0);


	 signal SRAM_DO            : std_logic_vector(7 downto 0);
	 
	 signal swnmi           	: std_logic;
	 signal swrst              : std_logic;
	 
	 signal joya               : std_logic_vector(6 downto 0);
	 signal joyb               : std_logic_vector(6 downto 0);
	 
	 -- Disk controller
	 signal cont_MAPn          : std_logic :='1';
	 signal cont_ROMDISn       : std_logic :='1';
    signal cont_D_OUT         : std_logic_vector(7 downto 0);
    signal cont_IOCONTROLn    : std_logic :='1';
	 signal cont_ECE           : std_logic;
    signal cont_nOE           : std_logic;
	 signal cont_irq           : std_logic;
	 
	
	 
	 -- Controller derived clocks
	 signal PH2_1              : std_logic;                                
    signal PH2_2              : std_logic;                                
    signal PH2_3              : std_logic;                                
    signal PH2_old            : std_logic_vector(3 downto 0);   
    signal PH2_cntr           : std_logic_vector(4 downto 0);
	 
COMPONENT keyboard
	PORT
	(
		clk_sys      : IN STD_LOGIC;
		key_pressed  : IN STD_LOGIC;
		key_extended : IN STD_LOGIC;
		key_strobe   : IN STD_LOGIC;
		key_code     : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		row          : IN STD_LOGIC_VECTOR(2 DOWNTO 0);
		col          : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
		key_hit      : OUT STD_LOGIC;
		swnmi        : OUT STD_LOGIC;
		swrst        : OUT STD_LOGIC
	);
END COMPONENT;

COMPONENT psg
			 PORT (
						clock : IN STD_LOGIC;
						ce    : IN STD_LOGIC;
						reset : IN STD_LOGIC;
						bdir : IN STD_LOGIC;
						bc1 : IN STD_LOGIC;
						sel : IN STD_LOGIC;
						d   : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
						q   : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
						
						ioad : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
					   ioaq : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
						iobd : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
						iobq : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
						
						MIX : OUT UNSIGNED (13 DOWNTO 0);
						A   : OUT UNSIGNED (11 DOWNTO 0);
						B   : OUT UNSIGNED(11 DOWNTO 0);
						C   : OUT UNSIGNED(11 DOWNTO 0)
			 );
END COMPONENT;

COMPONENT Microdisc
	PORT (
	  CLK_SYS : in std_logic;
	  DI : in std_logic_vector(7 DOWNTO 0);
	  DO : out std_logic_vector(7 DOWNTO 0);
	  A : in std_logic_vector(15 DOWNTO 0);
	  RnW : in std_logic;
	  nIRQ : out std_logic;
	  PH2 : in std_logic;
	  nROMDIS : out std_logic;
	  nMAP : out std_logic;
	  IO : in std_logic;
	  IOCTRL : out std_logic;
	  nHOSTRST : out std_logic;
	  nOE : out std_logic;
	  DIR : out std_logic;
	  nRESET : in std_logic;
	  nECE : out std_logic;
	  nEOE : out std_logic;
	  ENA : in std_logic;
	  img_mounted : in std_logic;
	  img_wp : in std_logic;
	  img_size : in std_logic_vector (31 DOWNTO 0);
	  sd_lba : out std_logic_vector (31 DOWNTO 0);
	  sd_rd : out std_logic;
	  sd_wr : out std_logic;
	  sd_ack : in std_logic;
	  sd_buff_addr : in std_logic_vector (8 DOWNTO 0);
	  sd_dout : in std_logic_vector (7 DOWNTO 0);
	  sd_din : out std_logic_vector (7 DOWNTO 0);
	  sd_dout_strobe : in std_logic;
	  sd_din_strobe : in std_logic;
	  fdd_ready : in std_logic;
	  fdd_layout : in std_logic;
	  fd_led : out std_logic
	);
END COMPONENT;



begin

RESETn <= (not RESET and KEYB_RESETn);
cpu_irq <= not via_irq and cont_irq;


cpu_rw <= cpu1_rw WHEN cpu_type /= "10" ELSE cpu2_rw;
cpu_do <= cpu1_do WHEN cpu_type /= "10" ELSE cpu2_do;
cpu_ad <= cpu1_ad WHEN cpu_type /= "10" ELSE cpu2_ad;

inst_cpu : entity work.T65
	port map (
		Mode    		=> cpu_type,
      Res_n   		=> RESETn,
      Enable  		=> ENA_1MHZ_N,
      Clk     		=> CLK_IN,
      Rdy     		=> '1',
      Abort_n 		=> '1',
      IRQ_n   		=> cpu_irq, -- Via and disk controller
      NMI_n   		=> KEYB_NMIn,
      SO_n    		=> '1',
      R_W_n   		=> cpu1_rw,
      A       		=> cpu1_ad,
      DI      		=> cpu_di,
      DO      		=> cpu1_do
);

	inst_cpu2 : ENTITY work.P65C816
		PORT MAP(
			CLK => clk_in, -- Mapeo del reloj de entrada
			RST_N => RESETn, -- Mapeo de la señal de reset activa baja
			CE => ENA_1MHZ_N, -- Clock Enable (similar a Enable en T65)
			RDY_IN => '1', -- Ready siempre activo ('1')
			NMI_N => KEYB_NMIn, -- NMI
			IRQ_N => cpu_irq, -- IRQ entrada, mapear con la señal de interrupción
			ABORT_N => '1', -- ABORT no utilizado ('1')
			D_IN => cpu_di, -- Datos de entrada
			D_OUT => cpu2_do, -- Datos de salida
			A_OUT => cpu2_ad, -- Dirección de salida
			WE => cpu2_rw, -- Write Enable (escribir/leer, similar a R_W_n)
			RDY_OUT => OPEN, -- No usamos RDY_OUT, por eso lo dejamos desconectado
			VPA => OPEN, -- Vector Address, no lo usamos, por eso lo desconectamos
			VDA => OPEN, -- Vector Data Address, no lo usamos
			MLB => OPEN, -- Memory Lock Byte, no utilizado
			VPB => OPEN -- Vector Pull, no utilizado
		);
		
--ram_ad  <= ula_AD_SRAM when (ula_PHI2 = '0')else cpu_ad(15 downto 0);
ram_ad  <= ula_AD_SRAM when (ula_PHI2 = '0') else cpu_ad(15 downto 0);


ram_d   <= cpu_do;
SRAM_DO <= ram_q;
ram_cs  <= '0' when RESETn = '0' else ula_CE_SRAM;
ram_oe  <= '0' when RESETn = '0' else ula_OE_SRAM;
ram_we  <= '0' when RESETn = '0' else ula_WE_SRAM;
phi2    <= ula_PHI2;

rom_ad  <= cpu_ad(15 downto 0);
rom_cs  <= '1' when ula_CSIOn = '1' and ula_CSROMn = '0' and cont_MAPn ='1' and cont_ROMDISn = '1' else '0';
rom_ext_cs <= '1' when ula_PHI2 = '1' and cont_ECE ='0' and cont_ROMDISn = '0' and cont_MAPn = '1' else '0'; -- Microdisc


inst_ula : entity work.ULA
   port map (
      CLK        	=> CLK_IN,
      PHI2       	=> ula_phi2,
      PHI2_EN       => ENA_1MHZ,
      PHI2_EN_N     => ENA_1MHZ_N,
      RW         	=> cpu_rw,
      RESETn     	=> pll_locked, --RESETn,
		MAPn      	=> cont_MAPn,
      DB         	=> SRAM_DO,
      ADDR       	=> cpu_ad(15 downto 0),
      SRAM_AD    	=> ula_AD_SRAM,
		SRAM_OE    	=> ula_OE_SRAM,
		SRAM_CE    	=> ula_CE_SRAM,
		SRAM_WE    	=> ula_WE_SRAM,
		LATCH_SRAM 	=> ula_LATCH_SRAM,
      CSIOn      	=> ula_CSIOn,
      CSROMn     	=> ula_CSROMn,
      CSRAMn     	=> ula_CSRAMn,
      R          	=> VIDEO_R,
      G          	=> VIDEO_G,
      B          	=> VIDEO_B,
      SYNC       	=> VIDEO_SYNC,
		BLANKn      => BLANKINGn,
		HSYNC      	=> VIDEO_HSYNC,
		VSYNC      	=> VIDEO_VSYNC,		
		HBLANK      => VIDEO_HBLANK,
		VBLANK      => VIDEO_VBLANK		
);

via_pa_out_oe_l <= not via_pa_out_oe;

inst_via6522 : entity work.via6522
	port map
	(
		clock           => CLK_IN,
		rising          => ENA_1MHZ,
		falling         => ENA_1MHZ_N,
		reset           => not RESETn,

		addr            => cpu_ad(3 downto 0),
		wen             => not cpu_rw and cont_IOCONTROLn and not ula_CSIOn,
		ren             => cpu_rw and cont_IOCONTROLn and not ula_CSIOn,
		data_in         => cpu_do,
		data_out        => VIA_DO,

		port_a_i        => via_pa_in,
		port_a_o        => via_pa_out,
		port_a_t        => via_pa_out_oe,

		port_b_i        => via_pb_in,
		port_b_o        => via_pb_out,
		port_b_t        => open,

		ca1_i           => '1',

		ca2_o           => via_ca2_out,
		ca2_i           => '1',
		ca2_t           => open,

		cb1_i           => K7_TAPEIN,
		cb1_o           => via_cb1_out,
		cb1_t           => open,

		cb2_i           => '1',
		cb2_o           => via_cb2_out,
		cb2_t           => open,

		irq             => via_irq
	);

  psg_a: psg
  port map (
    clock       => CLK_IN,
    ce          => ENA_1MHZ,
    reset       => RESETn AND KEYB_RESETn,
    bdir        => via_cb2_out,
    bc1         => via_ca2_out,
    d           => via_pa_out,
    q           => psg_do,
    a           => PSG_OUT_A,
    b           => PSG_OUT_B,
    c           => PSG_OUT_C,
	 mix         => PSG_OUT,
	 
	 ioad        => "ZZZZZZZZ",
	 ioaq        => ym_ioa_out,
	 iobd        => "ZZZZZZZZ",
	 iobq        => open,
	 
    sel         => '1'
    );

inst_key : keyboard
	port map(
		clk_sys      => CLK_IN,
		key_pressed  => key_pressed,
		key_extended => key_extended,
		key_strobe   => key_strobe,
		key_code     => key_code,
		row          => via_pb_out (2 downto 0),
		col          => ym_ioa_out,
		key_hit      => KEY_HIT,
		swnmi        => swnmi,
		swrst        => swrst
);

KEYB_NMIn <= NOT swnmi;
KEYB_RESETn <= NOT swrst;

inst_microdisc: component Microdisc 
    port map( 
          CLK_SYS   => CLK_IN,
                                                            -- Oric Expansion Port Signals
          DI        => cpu_do,                              -- 6502 Data Bus
          DO        => cont_D_OUT,                          -- 6502 Data Bus			 
          A         => cpu_ad (15 downto 0),                -- 6502 Address Bus
          RnW       => cpu_rw,                              -- 6502 Read-/Write
          nIRQ      => cont_irq,                            -- 6502 /IRQ
          PH2       => ula_PHI2,                            -- 6502 PH2 
          nROMDIS   => cont_ROMDISn,                        -- Oric ROM Disable
          nMAP      => cont_MAPn,                           -- Oric MAP 
          IO        => ula_CSIOn,                           -- Oric I/O 
          IOCTRL    => cont_IOCONTROLn,                     -- Oric I/O Control           
                                                            -- Additional MCU Interface Lines
          nRESET    => RESETn,                              -- RESET from MCU
          --DSEL      => cont_DSEL,                           -- Drive Select
          --SSEL      => cont_SSEL,                           -- Side Select
          
                                                             -- EEPROM Control Lines.
          nECE      => cont_ECE,                             -- Chip Enable
 
			 ENA       => disk_enable,
			 
			 nOE       => cont_nOE,
			 
			 img_mounted    => img_mounted,
			 img_wp         => img_wp,
			 img_size       => img_size,
			 sd_lba         => sd_lba,
			 sd_rd          => sd_rd,
			 sd_wr          => sd_wr,
			 sd_ack         => sd_ack,
			 sd_buff_addr   => sd_buff_addr,
			 sd_dout        => sd_dout,
			 sd_din         => sd_din,
			 sd_dout_strobe => sd_dout_strobe,
			 sd_din_strobe  => sd_din_strobe,
			 fdd_ready      => fdd_ready,
			 fdd_layout     => fdd_layout,
			 fd_led         => fd_led
			 
         );



via_pa_in <= (via_pa_out and not via_pa_out_oe_l) or (psg_do and via_pa_out_oe_l);
via_pb_in(2 downto 0) <= via_pb_out(2 downto 0);
via_pb_in(3) <= KEY_HIT;
via_pb_in(4) <=via_pb_out(4);
via_pb_in(5) <= 'Z';
via_pb_in(6) <=via_pb_out(6);
via_pb_in(7) <=via_pb_out(7);



K7_TAPEOUT  <= via_pb_out(7);
K7_REMOTE   <= via_pb_out(6);
PRN_STROBE  <= via_pb_out(4);
PRN_DATA    <= via_pa_out;


--joya <= joystick_0(6 downto 4) & joystick_0(0) & joystick_0(1) & joystick_0(2) & joystick_0(3);
--joyb <= joystick_1(6 downto 4) & joystick_1(0) & joystick_1(1) & joystick_1(2) & joystick_1(3);

cpu_di <= cont_D_OUT when ula_CSIOn = '0' and cont_IOCONTROLn = '0' else -- expansion port
          VIA_DO     when ula_CSIOn = '0' and cont_IOCONTROLn = '1' else -- VIA
          rom_q when ula_CSIOn = '1' and ula_CSROMn = '0' and cont_MAPn ='1' and cont_ROMDISn = '1' else  -- ROM Oric 1 or Atmos
          rom_q when cont_ECE ='0' and cont_ROMDISn = '0' and cont_MAPn = '1' else --ROM Microdisc
          SRAM_DO when ula_CSRAMn = '0' else -- RAM
		    cpu_di_last;

process (CLK_IN) begin
	if rising_edge(CLK_IN) then
		if cpu_rw = '1' and ula_phi2 = '1' then
			cpu_di_last <= cpu_di;
		end if;
	end if;
end process;

end RTL;
