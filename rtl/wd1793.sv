// ====================================================================
//
//  WD1793 replica (with write capability), MFM_DISK image backend
//
//  Copyright (C) 2007,2008 Viacheslav Slavinsky
//  Copyright (C) 2016 Sorgelig
//  Copyright (C) 2026 rampa (Oricutron MFM_DISK backend)
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================
//
// Derived from the wd1793.sv used by the MiST Oric core. The register file,
// the command decoder, the status bits and the sector transfer through the
// 2 KB block buffer are kept. The EDSK image scan, the fixed geometry modes
// and the RAM backend are replaced by a reader for the MFM_DISK images used
// by Oricutron:
//
//   0x000  "MFM_DISK", sides (LE32 at 8), tracks (LE32 at 12),
//          geometry (LE32 at 16, only 1 is supported, as in Oricutron)
//   0x100  raw tracks of 6400 bytes, all side 0 tracks first, so a track
//          starts at 256 + (side * tracks + track) * 6400
//
// A track holds the decoded MFM stream with the sync bytes stored as
// A1 A1 A1, the ID field as A1 A1 A1 FE C H R N CRC CRC and the data field
// as A1 A1 A1 FB (or F8 when deleted), data, CRC CRC. The CRC is CRC-CCITT
// preset to FFFF over the sync bytes, the mark and the field.
//
// When a Type II or Type III command needs the current track and side, the
// 13 SD blocks holding that track are read and parsed into a table of up to
// 32 sectors (ID field, data offset inside the track, deleted mark). The
// table stays valid until the head moves, the side changes or a new image is
// mounted. The sector data is then read and written in place through the
// block buffer, and a sector write also updates the data CRC in the image.
//
// Write Sector writes the data mark asked by a0 (F8 deleted, FB normal) and
// the data CRC. Read Track returns the 6400 raw bytes of the track and Write
// Track rewrites them, F5 becoming A1 with a CRC preset, F6 becoming C2 and
// F7 the two CRC bytes; the index of the image track ends both commands, and
// Write Track can only format tracks the image file already holds. With
// check_crc high, ID fields with a bad CRC are skipped and a bad data CRC
// ends a read with the CRC error bit. It is off by default because images
// made by old tools (the Sedoric 3 master among them) store F7 F7 instead of
// the CRCs, and Oricutron never checks them.
//
// ready is the READY line of the drive. With ready high and no image mounted
// the drive spins empty: there is no index pulse and no ID field, so Type II
// and Type III commands wait, busy, until an image is mounted (then they go
// on as with a disk just inserted) or a Force Interrupt ends them. The Oric
// Microdisc boot ROM relies on this to stay on "insert system disc".

module wd1793 #(
	parameter STRICT_ID    = 0,        // 1: the ID track must match the track register
	parameter INDEX_PERIOD = 800000,   // ce ticks per turn: 200 ms (300 rpm) at 4 MHz
	parameter INDEX_WIDTH  = 8000      // ce ticks of index pulse: 2 ms at 4 MHz
)
(
	input        clk_sys,     // sys clock
	input        ce,          // FDC clock enable, at most clk_sys / 2
	input        reset,       // sync reset
	input        io_en,
	input        rd,          // i/o read
	input        wr,          // i/o write
	input  [1:0] addr,        // i/o port addr
	input  [7:0] din,         // i/o data in
	output [7:0] dout,        // i/o data out
	output       drq,         // DMA request
	output       intrq,
	output       busy,
	output       headloaded,

	input        wp,          // write protect
	input        side,
	input        ready,
	input        check_crc,   // 1: check the ID and data CRCs (see above)

	input        img_mounted, // signaling that new image has been mounted
	input [31:0] img_size,    // size of image in bytes
	output[31:0] sd_lba,
	output reg   sd_rd,
	output reg   sd_wr,
	input        sd_ack,
	input  [8:0] sd_buff_addr,
	input  [7:0] sd_buff_dout,
	output [7:0] sd_buff_din,
	input        sd_buff_wr
);

localparam TRACK_LEN  = 6400;
localparam DAM_WINDOW = 43;    // bytes after the ID CRC where the data mark must start (MFM)

assign dout       = q;
assign drq        = s_drq;
assign busy       = s_busy;
assign intrq      = s_intrq;
assign headloaded = s_headloaded;
assign sd_lba     = {9'd0, scan_mode ? scan_lba : buff_a[23:9] + sd_block};

wire [10:0] sector_size = 11'd128 << wd_size_code;
reg  [10:0] byte_addr;
reg  [23:0] buff_a;         // image offset of the sector data
reg   [1:0] wd_size_code;
reg   [1:0] sd_block = 0;

reg         scan_mode = 0;      // header or track scan owns the block buffer
reg  [14:0] scan_lba;
reg   [8:0] scan_ptr;

// byte written into the block buffer by the state machine (data mark, CRC,
// Write Track) at the address the buffer port B points to
reg         fsm_wr = 0;
reg   [7:0] fsm_byte;

wire  [7:0] buff_dout;
reg         buff_wr;

wd1793_dpram sbuf
(
	.clock(clk_sys),

	.address_a({sd_block, sd_buff_addr}),
	.data_a(sd_buff_dout),
	.wren_a(sd_buff_wr & sd_ack),
	.q_a(sd_buff_din),

	.address_b(scan_mode ? {2'b00, scan_ptr} : byte_addr),
	.data_b(fsm_wr ? fsm_byte : din),
	.wren_b(fsm_wr | (wre & buff_wr & (addr == A_DATA) & ~scan_mode)),
	.q_b(buff_dout)
);

// Image geometry, read from the header on the first access after a mount
reg  [23:0] disk_size = 0;
reg         hdr_loaded = 0, hdr_valid = 0;
wire        disk_in = |disk_size;   // an image is in the drive
reg   [7:0] hdr_sides, hdr_tracks;

wire        trk_exists = hdr_valid && (disk_track < hdr_tracks) && (!side || hdr_sides == 2);
wire  [8:0] trk_index  = {1'b0, disk_track} + (side ? {1'b0, hdr_tracks} : 9'd0);
reg  [23:0] trk_base;
always @(posedge clk_sys) trk_base <= 24'd256 + trk_index * 24'd6400;

// Sector table of the cached track:
// {C, H, R, N, ID CRC, deleted, has data, data offset in the track}
reg  [56:0] sect_tab[32];
reg  [56:0] tab_q, tab_wd;
reg   [4:0] tab_ra, tab_wa;
reg         tab_we = 0;
always @(posedge clk_sys) begin
	if(tab_we) sect_tab[tab_wa] <= tab_wd;
	tab_q <= sect_tab[tab_ra];
end

wire  [7:0] e_c     = tab_q[56:49];
wire  [7:0] e_h     = tab_q[48:41];
wire  [7:0] e_r     = tab_q[40:33];
wire  [1:0] e_n     = tab_q[32:31];
wire [15:0] e_crc   = tab_q[30:15];
wire        e_del   = tab_q[14];
wire        e_has   = tab_q[13];
wire [12:0] e_off   = tab_q[12:0];

reg         cache_valid = 0;
reg   [7:0] cache_track;
reg         cache_side;
reg   [5:0] n_sect = 0;
reg   [4:0] sec_idx = 0;        // last sector passed under the head
wire  [4:0] next_ra = ({1'b0, tab_ra} + 1'd1 >= n_sect) ? 5'd0 : tab_ra + 1'd1;

// CRC-CCITT, one byte
function [15:0] crc_next(input [15:0] c, input [7:0] b);
	integer i;
	reg [15:0] r;
	begin
		r = c ^ {b, 8'h00};
		for(i = 0; i < 8; i = i + 1) r = r[15] ? {r[14:0], 1'b0} ^ 16'h1021 : {r[14:0], 1'b0};
		crc_next = r;
	end
endfunction

// Register addresses
localparam A_COMMAND         = 0;
localparam A_STATUS          = 0;
localparam A_TRACK           = 1;
localparam A_SECTOR          = 2;
localparam A_DATA            = 3;

// Track parser states
localparam P_HUNT = 0, P_ID = 1, P_DAM = 2, P_SKIP = 3;

// States
typedef enum
{
	STATE_IDLE,

	STATE_SEARCH,
	STATE_SEARCH_CHK,
	STATE_SEARCH_0,
	STATE_SEARCH_1,
	STATE_SEARCH_2,

	STATE_LOAD_1,
	STATE_LOAD_2,

	STATE_HDR,
	STATE_HDR_A,
	STATE_HDR_D,
	STATE_HDR_END,

	STATE_SCAN,
	STATE_SCAN_BLK,
	STATE_SCAN_A,
	STATE_SCAN_D,
	STATE_SCAN_END,

	STATE_WAIT_READ,
	STATE_WAIT_READ_1,
	STATE_WAIT_READ_2,

	STATE_READ,
	STATE_READ_1,
	STATE_READ_2,

	STATE_WAIT_WRITE,
	STATE_WAIT_WRITE_1,
	STATE_WAIT_WRITE_2,

	STATE_WRITE,
	STATE_WRITE_1,
	STATE_WRITE_2,
	STATE_WRITE_CRC1,
	STATE_WRITE_CRC2,
	STATE_WRITE_CRC3,
	STATE_WRITE_MARK,
	STATE_WRITE_MARK_2,

	STATE_READ_CRC_1,
	STATE_READ_CRC_2,
	STATE_READ_CRC_3,
	STATE_READ_NEXT,

	STATE_RT,
	STATE_RT_BLK,
	STATE_RT_BYTE,
	STATE_RT_1,
	STATE_RT_2,

	STATE_WT,
	STATE_WT_RESUME,
	STATE_WT_REQ,
	STATE_WT_REQ_1,
	STATE_WT_WAIT,
	STATE_WT_EMIT,
	STATE_WT_ADV,
	STATE_WT_FLUSH,
	STATE_WT_FLUSH_2,

	STATE_ABORT,
	STATE_WAIT,
	STATE_WAIT_2,
	STATE_ENDCOMMAND
} io_state_t;


// common status bits
wire        s_readonly = wp;
reg			s_crcerr;
reg			s_headloaded, s_seekerr, s_index;  // mode 1
reg			s_lostdata, s_wrfault; 			     // mode 2,3 (s_wrfault is the record type on reads)

// Command mode 0/1 for status register
reg 			cmd_mode;

// allow write protect flag
reg 			s_wpe;

// DRQ/BUSY are always going together
reg	[1:0]	s_drq_busy;
wire			s_drq  = s_drq_busy[1];
wire			s_busy = s_drq_busy[0];
reg         s_intrq;

reg   [7:0] wdreg_track;
reg   [7:0] wdreg_sector;
reg   [7:0] wdreg_data;
wire  [7:0] wdreg_status = cmd_mode == 0 ?
	{~ready, s_readonly & s_wpe, s_headloaded, s_seekerr | ~ready, s_crcerr, !disk_track, s_index, s_busy}:
	{~ready, s_readonly & s_wpe, s_wrfault,    s_seekerr | ~ready, s_crcerr, s_lostdata,  s_drq,   s_busy};

reg   [7:0] read_addr[6];
reg   [7:0] q;
always @* begin
	case (addr)
		A_STATUS: q = wdreg_status;
		A_TRACK:  q = wdreg_track;
		A_SECTOR: q = wdreg_sector;
		A_DATA:   q = (state == STATE_IDLE) ? wdreg_data : buff_rd ? buff_dout : read_addr[byte_addr[2:0]];
	endcase
end

reg         buff_rd;
reg         step_direction; // last step direction

reg   [7:0] disk_track;		 // "real" heads position
reg  [10:0]	data_length;	 // this many bytes to transfer during read/write ops
io_state_t  state = STATE_IDLE;

// Reusable expressions
wire  [7:0] next_track  = (din[6] ? din[5] : step_direction) ? disk_track - 1'd1 : disk_track + 1'd1;
wire [10:0]	next_length = data_length - 1'b1;

// Watchdog
reg         watchdog_set;
wire        watchdog_bark = (wd_timer == 0);
reg  [15:0] wd_timer;
always @(posedge clk_sys) begin
	if(ce) begin
		if(watchdog_set) wd_timer <= 4096;
			else if(wd_timer != 0) wd_timer <= wd_timer - 1'b1;
	end
end

always @(posedge clk_sys) begin
	integer cnt;
	if(ce) begin
		if(ready && disk_in) begin
			if(cnt) cnt <= cnt - 1;
				else cnt <= INDEX_PERIOD - 1;
		end else cnt <= 0;
		s_index <= ready && disk_in && (cnt < INDEX_WIDTH);
	end
end

wire        rde = rd & io_en;
wire        wre = wr & io_en;

// last byte of the transfer kept in the block buffer: the data, plus the
// CRC when it is written or checked, plus the data mark on writes (buff_a then
// points to the mark)
wire [11:0] span_last = {3'd0, buff_a[8:0]} + {1'b0, sector_size} +
                        (write ? 12'd2 : check_crc ? 12'd1 : 12'hFFF);

// CRC of the ID field in the sector table entry, from A1 A1 A1 FE
wire [15:0] e_id_crc = crc_next(crc_next(crc_next(crc_next(16'hB230, e_c), e_h), e_r), {6'd0, e_n});

reg         write;
always @(posedge clk_sys) begin
	reg old_wr, old_rd;

	reg [2:0] cur_addr;
	reg       read_data;
	reg       write_data;
	reg       rw_type;
	integer   wait_time;
	reg [3:0] read_timer;
	reg [9:0] seektimer;
	reg       multisector;
	reg [5:0] ack;
	reg       sd_busy;
	reg       old_mounted;
	reg [1:0] blk_max;
	io_state_t load_ret;

	reg [15:0] crc;
	reg  [6:0] srch_cnt;

	reg  [4:0] hp;
	reg        hdr_bad;

	reg [12:0] tpos;
	reg [23:0] scan_off;
	reg  [1:0] pstate;
	reg [23:0] sync;
	reg  [2:0] pcnt;
	reg  [7:0] id_c, id_h, id_r;
	reg  [1:0] id_n;
	reg [15:0] id_crc;
	reg  [5:0] win;
	reg [10:0] skip;

	reg        wr_del;          // a0 of Write Sector: deleted data mark
	io_state_t hdr_ret;
	reg  [7:0] wt_b1, wt_b2;    // Write Track bytes still to store
	reg  [1:0] wt_n;
	reg        wt_end, last_f5;

	if(reset) begin
		read_data <= 0;
		write_data <= 0;
		multisector <= 0;
		step_direction <= 0;
		disk_track <= 0;
		wdreg_track <= 0;
		wdreg_sector <= 0;
		wdreg_data <= 0;
		data_length <= 0;
		byte_addr <=0;
		buff_rd <= 0;
		buff_wr <= 0;
		state <= STATE_IDLE;
		cmd_mode <= 0;
		s_wpe <= 1;
		{s_headloaded, s_seekerr, s_crcerr, s_intrq} <= 0;
		{s_wrfault, s_lostdata} <= 0;
		s_drq_busy <= 0;
		watchdog_set <= 0;
		seektimer <= 'h3FF;
		{ack, sd_wr, sd_rd, sd_busy} <= 0;
		scan_mode <= 0;
		fsm_wr <= 0;
		tab_we <= 0;
		sec_idx <= 0;
		cache_valid <= 0;
	end else if(ce) begin

		ack <= {ack[4:0], sd_ack};
		if(ack[5:4] == 'b01) {sd_rd,sd_wr} <= 0;
		if(ack[5:4] == 'b10) sd_busy <= 0;

		tab_we <= 0;

		old_wr <=wre;
		old_rd <=rde;

		if((!old_rd && rde) || (!old_wr && wre)) cur_addr <= addr;

		//Register read operations
		if(old_rd && !rde && (cur_addr == A_STATUS)) s_intrq <= 0;

		//end of data reading
		if(old_rd && !rde && (cur_addr == A_DATA)) read_data <=1;

		//end of data writing
		if(old_wr && !wre && (cur_addr == A_DATA)) write_data <=1;

		case (state)
			/* Idle state or buffer to host transfer */
			STATE_IDLE:; // do nothing

			STATE_SEARCH:
				begin
					if(!ready) begin
						s_seekerr <= 1;
						state <= STATE_ENDCOMMAND;
					end
					else if(!disk_in) ; // empty drive: wait for an image
					else begin
						seektimer <= seektimer - 1'b1;
						if(!seektimer) state <= STATE_SEARCH_CHK;
					end
				end

			// header, then track cache, then the sector table
			STATE_SEARCH_CHK:
				begin
					if(!hdr_loaded) begin
						hdr_ret <= STATE_SEARCH_CHK;
						state   <= STATE_HDR;
					end
					else if(!hdr_valid) begin
						s_seekerr <= 1;
						state <= STATE_ENDCOMMAND;
					end
					else if(!cache_valid || (cache_track != disk_track) || (cache_side != side)) state <= STATE_SCAN;
					else state <= STATE_SEARCH_0;
				end

			STATE_SEARCH_0:
				begin
					if(!n_sect) begin
						s_seekerr <= 1;           // record not found
						state <= STATE_ENDCOMMAND;
					end else begin
						srch_cnt <= {n_sect, 1'b0};   // two revolutions, as Oricutron
						tab_ra   <= ({1'b0, sec_idx} + 1'd1 >= n_sect) ? 5'd0 : sec_idx + 1'd1;
						state    <= STATE_SEARCH_1;
					end
				end
			STATE_SEARCH_1: state <= STATE_SEARCH_2; // sector table read latency
			STATE_SEARCH_2:
				begin
					sec_idx <= tab_ra;
					if(~rw_type) begin
						// READ ADDRESS: the next ID field under the head
						read_addr[0] <= e_c;
						read_addr[1] <= e_h;
						read_addr[2] <= e_r;
						read_addr[3] <= {6'd0, e_n};
						read_addr[4] <= e_crc[15:8];
						read_addr[5] <= e_crc[7:0];
						wdreg_sector <= e_c;      // the ID track goes to the sector register
						byte_addr    <= 0;
						if(check_crc && (e_id_crc != e_crc)) s_crcerr <= 1;
						state        <= STATE_READ;
					end
					else if(e_has && (e_r == wdreg_sector) && (!STRICT_ID || (e_c == wdreg_track)) &&
					        !(check_crc && (e_id_crc != e_crc))) begin
						s_crcerr     <= 0;
						wd_size_code <= e_n;
						if(write) begin
							// start at the data mark, which is rewritten, and
							// record the new mark in the sector table
							buff_a <= trk_base + e_off - 1'd1;
							crc    <= wr_del ? 16'hD2F6 : 16'hE295;   // CRC of A1 A1 A1 F8 / FB
							tab_we <= 1;
							tab_wa <= tab_ra;
							tab_wd <= {tab_q[56:15], wr_del, tab_q[13:0]};
						end else begin
							buff_a    <= trk_base + e_off;
							crc       <= e_del ? 16'hD2F6 : 16'hE295;
							s_wrfault <= e_del;                         // record type
						end
						state        <= STATE_WAIT_READ;
					end
					else if(srch_cnt == 1) begin
						s_seekerr <= 1;           // record not found
						state <= STATE_ENDCOMMAND;
					end
					else begin
						if(check_crc && (e_id_crc != e_crc)) s_crcerr <= 1;
						srch_cnt <= srch_cnt - 1'd1;
						tab_ra   <= next_ra;
						state    <= STATE_SEARCH_1;
					end
				end

			// read one block at scan_lba into slot 0 of the block buffer
			STATE_LOAD_1:
				begin
					sd_busy <= 1;
					sd_rd   <= 1;
					state   <= STATE_LOAD_2;
				end
			STATE_LOAD_2: if(!sd_busy) state <= load_ret;

			// image header
			STATE_HDR:
				begin
					scan_mode <= 1;
					scan_lba  <= 0;
					sd_block  <= 0;
					hdr_bad   <= 0;
					hp        <= 0;
					load_ret  <= STATE_HDR_A;
					state     <= (disk_size < 20) ? STATE_HDR_END : STATE_LOAD_1;
					if(disk_size < 20) hdr_bad <= 1;
				end
			STATE_HDR_A:
				begin
					scan_ptr <= {4'd0, hp};
					state    <= STATE_HDR_D; // block buffer read latency
				end
			STATE_HDR_D:
				begin
					// scan_ptr was set two ce ago, buff_dout holds the byte
					case(hp)
						0,1,2,3,4,5,6,7: if(buff_dout != mfm_sig(hp[2:0])) hdr_bad <= 1;
						8:  hdr_sides  <= buff_dout;
						12: hdr_tracks <= buff_dout;
						16: if(buff_dout != 1) hdr_bad <= 1;
						default: if(buff_dout) hdr_bad <= 1;
					endcase
					hp <= hp + 1'd1;
					state <= (hp == 19) ? STATE_HDR_END : STATE_HDR_A;
				end
			STATE_HDR_END:
				begin
					scan_mode  <= 0;
					hdr_loaded <= 1;
					hdr_valid  <= ~hdr_bad && (hdr_sides == 1 || hdr_sides == 2) && hdr_tracks && (hdr_tracks <= 128);
					state      <= hdr_ret;
				end

			// track scan into the sector table
			STATE_SCAN:
				begin
					scan_mode   <= 1;
					sd_block    <= 0;
					n_sect      <= 0;
					sec_idx     <= 0;
					cache_track <= disk_track;
					cache_side  <= side;
					pstate      <= P_HUNT;
					sync        <= 0;
					tpos        <= 0;
					scan_lba    <= trk_base[23:9];
					scan_ptr    <= trk_base[8:0];
					scan_off    <= trk_base;
					load_ret    <= STATE_SCAN_A;
					state       <= trk_exists ? STATE_SCAN_BLK : STATE_SCAN_END;
				end
			STATE_SCAN_BLK: state <= ({scan_lba, 9'd0} >= disk_size) ? STATE_SCAN_END : STATE_LOAD_1;
			STATE_SCAN_A:   state <= STATE_SCAN_D; // block buffer read latency
			STATE_SCAN_D:
				begin
					case(pstate)
						P_HUNT:
							begin
								sync <= {sync[15:0], buff_dout};
								if((sync == 24'hA1A1A1) && (buff_dout == 8'hFE)) begin
									pstate <= P_ID;
									pcnt   <= 0;
								end
							end
						P_ID:
							begin
								pcnt <= pcnt + 1'd1;
								case(pcnt)
									0: id_c <= buff_dout;
									1: id_h <= buff_dout;
									2: id_r <= buff_dout;
									3: id_n <= buff_dout[1:0];
									4: id_crc[15:8] <= buff_dout;
									5: begin
											id_crc[7:0] <= buff_dout;
											sync   <= 0;
											win    <= 0;
											pstate <= P_DAM;
										end
									default:;
								endcase
							end
						P_DAM:
							begin
								sync <= {sync[15:0], buff_dout};
								win  <= win + 1'd1;
								if((sync == 24'hA1A1A1) && ((buff_dout == 8'hFB) || (buff_dout == 8'hF8))) begin
									if(!n_sect[5]) begin
										tab_we <= 1;
										tab_wa <= n_sect[4:0];
										tab_wd <= {id_c, id_h, id_r, id_n, id_crc, buff_dout == 8'hF8, 1'b1, tpos + 1'd1};
										n_sect <= n_sect + 1'd1;
									end
									skip   <= (11'd128 << id_n) + 11'd2;  // data and CRC
									pstate <= P_SKIP;
								end
								else if(win == DAM_WINDOW) begin
									// ID field without data field
									if(!n_sect[5]) begin
										tab_we <= 1;
										tab_wa <= n_sect[4:0];
										tab_wd <= {id_c, id_h, id_r, id_n, id_crc, 1'b0, 1'b0, 13'd0};
										n_sect <= n_sect + 1'd1;
									end
									sync   <= 0;
									pstate <= P_HUNT;
								end
							end
						P_SKIP:
							begin
								skip <= skip - 1'd1;
								if(skip == 1) begin
									sync   <= 0;
									pstate <= P_HUNT;
								end
							end
					endcase

					tpos     <= tpos + 1'd1;
					scan_off <= scan_off + 1'd1;
					scan_ptr <= scan_ptr + 1'd1;
					if((tpos == TRACK_LEN - 1) || (scan_off + 1'd1 >= disk_size)) state <= STATE_SCAN_END;
					else if(&scan_ptr) begin
						scan_lba <= scan_lba + 1'd1;
						state    <= STATE_SCAN_BLK;
					end
					else state <= STATE_SCAN_A;
				end
			STATE_SCAN_END:
				begin
					scan_mode   <= 0;
					cache_valid <= 1;
					state       <= STATE_SEARCH_0;
				end

			// read before write, the sector is rarely aligned to the SD blocks
			STATE_WAIT_READ:
				begin
					data_length <= sector_size;
					byte_addr   <= {2'b00, buff_a[8:0]};
					blk_max     <= span_last[10:9];
					sd_block    <= 0;
					state       <= STATE_WAIT_READ_1;
				end
			STATE_WAIT_READ_1:
				begin
					sd_busy <= 1;
					sd_rd   <= 1;
					state   <= STATE_WAIT_READ_2;
				end
			STATE_WAIT_READ_2:
				begin
					if(!sd_busy) begin
						sd_block <= sd_block + 1'd1;
						state <= write ? STATE_WRITE_MARK : STATE_READ;
						if(sd_block < blk_max) state <= STATE_WAIT_READ_1;
					end
				end

			STATE_READ:
				begin
					watchdog_set <= 1;
					read_timer <= 15;
					state <= STATE_READ_1;
				end
			STATE_READ_1:
				begin
					read_timer <= read_timer - 1'b1;
					if(!read_timer) begin
						read_data <= 0;
						watchdog_set <= 0;
						s_lostdata <= 0;
						s_drq_busy <= 2'b11;
						state <= STATE_READ_2;
					end
				end
			STATE_READ_2:
				begin
					if(watchdog_bark | (read_data & s_drq)) begin
						// reset drq until next byte is read, nothing is lost
						s_drq_busy <= 2'b01;
						s_lostdata <= watchdog_bark;
						crc <= crc_next(crc, buff_dout);

						if(next_length == 0) begin
							state <= (rw_type && check_crc) ? STATE_READ_CRC_1 : STATE_READ_NEXT;
						end else begin
							byte_addr <= byte_addr + 1'd1;
							data_length <= next_length;
							state <= STATE_READ;
						end
					end
				end

			// data CRC check: the two bytes after the data
			STATE_READ_CRC_1:
				begin
					byte_addr <= byte_addr + 1'd1;
					state     <= STATE_READ_CRC_2;
				end
			STATE_READ_CRC_2:
				begin
					if(buff_dout != crc[15:8]) s_crcerr <= 1;
					byte_addr <= byte_addr + 1'd1;
					state     <= STATE_READ_CRC_3;
				end
			STATE_READ_CRC_3:
				begin
					if(buff_dout != crc[7:0]) s_crcerr <= 1;
					state <= STATE_READ_NEXT;
				end
			// either read the next sector, or stop if this is track end
			STATE_READ_NEXT:
				begin
					if(multisector && !s_crcerr) begin
						wdreg_sector <= wdreg_sector + 1'b1;
						state <= STATE_SEARCH;
					end else begin
						state <= STATE_ENDCOMMAND;
					end
				end

			STATE_WAIT_WRITE:
				begin
					if(!ready) begin
						s_wrfault <= 1;
						state <= STATE_ENDCOMMAND;
					end else begin
						sd_block <= 0;
						state <= STATE_WAIT_WRITE_1;
					end
				end
			STATE_WAIT_WRITE_1:
				begin
					sd_busy <= 1;
					sd_wr   <= 1;
					state   <= STATE_WAIT_WRITE_2;
				end
			STATE_WAIT_WRITE_2:
				begin
					if(!sd_busy) begin
						sd_block <= sd_block + 1'd1;
						if(sd_block < blk_max) state <= STATE_WAIT_WRITE_1;
						else if(multisector) begin
							wdreg_sector <= wdreg_sector + 1'b1;
							state <= STATE_SEARCH;
						end else begin
							state <= STATE_ENDCOMMAND;
						end
					end
				end
			STATE_WRITE:
				begin
					watchdog_set <= 1;
					read_timer <= 15;
					state <= STATE_WRITE_1;
				end
			STATE_WRITE_1:
				begin
					read_timer <= read_timer - 1'b1;
					if(!read_timer) begin
						write_data <= 0;
						watchdog_set <= 0;
						s_lostdata <= 0;
						s_drq_busy <= 2'b11;
						state <= STATE_WRITE_2;
					end
				end
			STATE_WRITE_2:
				begin
					if(watchdog_bark | (write_data & s_drq)) begin
						s_drq_busy <= 2'b01;
						s_lostdata <= watchdog_bark;
						crc <= crc_next(crc, wdreg_data);

						if(!next_length) state <= STATE_WRITE_CRC1;
						else begin
							byte_addr <= byte_addr + 1'd1;
							data_length <= next_length;
							state <= STATE_WRITE;
						end
					end
				end
			// data CRC after the last byte, then the blocks go back to the image
			STATE_WRITE_CRC1:
				begin
					byte_addr <= byte_addr + 1'd1;
					fsm_byte  <= crc[15:8];
					fsm_wr    <= 1;
					state     <= STATE_WRITE_CRC2;
				end
			STATE_WRITE_CRC2:
				begin
					byte_addr <= byte_addr + 1'd1;
					fsm_byte  <= crc[7:0];
					state     <= STATE_WRITE_CRC3;
				end
			STATE_WRITE_CRC3:
				begin
					fsm_wr <= 0;
					state  <= STATE_WAIT_WRITE;
				end
			// data mark in front of the data
			STATE_WRITE_MARK:
				begin
					fsm_byte <= wr_del ? 8'hF8 : 8'hFB;
					fsm_wr   <= 1;
					state    <= STATE_WRITE_MARK_2;
				end
			STATE_WRITE_MARK_2:
				begin
					fsm_wr    <= 0;
					byte_addr <= byte_addr + 1'd1;
					state     <= STATE_WRITE;
				end

			// READ TRACK: the raw bytes of the image track, block by block
			STATE_RT:
				begin
					if(!ready) state <= STATE_ENDCOMMAND;
					else if(!disk_in) ; // empty drive: wait for an image
					else if(!hdr_loaded) begin
						hdr_ret <= STATE_RT;
						state   <= STATE_HDR;
					end
					else if(!trk_exists) state <= STATE_ENDCOMMAND;
					else begin
						scan_mode <= 1;
						buff_rd   <= 1;
						sd_block  <= 0;
						scan_lba  <= trk_base[23:9];
						scan_ptr  <= trk_base[8:0];
						tpos      <= 0;
						state     <= STATE_RT_BLK;
					end
				end
			STATE_RT_BLK:
				begin
					load_ret <= STATE_RT_BYTE;
					state    <= ({scan_lba, 9'd0} >= disk_size) ? STATE_ENDCOMMAND : STATE_LOAD_1;
				end
			STATE_RT_BYTE:
				begin
					watchdog_set <= 1;
					read_timer   <= 15;
					state        <= STATE_RT_1;
				end
			STATE_RT_1:
				begin
					read_timer <= read_timer - 1'b1;
					if(!read_timer) begin
						read_data    <= 0;
						watchdog_set <= 0;
						s_drq_busy   <= 2'b11;
						state        <= STATE_RT_2;
					end
				end
			STATE_RT_2:
				begin
					if(watchdog_bark | (read_data & s_drq)) begin
						s_drq_busy <= 2'b01;
						if(watchdog_bark) s_lostdata <= 1;
						tpos     <= tpos + 1'd1;
						scan_ptr <= scan_ptr + 1'd1;
						if(tpos == TRACK_LEN - 1) state <= STATE_ENDCOMMAND;
						else if(&scan_ptr) begin
							scan_lba <= scan_lba + 1'd1;
							state    <= STATE_RT_BLK;
						end
						else state <= STATE_RT_BYTE;
					end
				end

			// WRITE TRACK: each block of the image track is read, overwritten
			// with the translated bytes and written back
			STATE_WT:
				begin
					if(!ready) state <= STATE_ENDCOMMAND;
					else if(!disk_in) ; // empty drive: wait for an image
					else if(!hdr_loaded) begin
						hdr_ret <= STATE_WT;
						state   <= STATE_HDR;
					end
					else if(!trk_exists || ({trk_base[23:9], 9'd0} >= disk_size)) begin
						s_wrfault <= 1;
						state     <= STATE_ENDCOMMAND;
					end
					else begin
						scan_mode   <= 1;
						sd_block    <= 0;
						scan_lba    <= trk_base[23:9];
						scan_ptr    <= trk_base[8:0];
						tpos        <= 0;
						wt_n        <= 0;
						wt_end      <= 0;
						last_f5     <= 0;
						cache_valid <= 0;
						load_ret    <= STATE_WT_RESUME;
						state       <= STATE_LOAD_1;
					end
				end
			STATE_WT_RESUME: state <= wt_n ? STATE_WT_EMIT : STATE_WT_REQ;
			STATE_WT_REQ:
				begin
					watchdog_set <= 1;
					read_timer   <= 15;
					state        <= STATE_WT_REQ_1;
				end
			STATE_WT_REQ_1:
				begin
					read_timer <= read_timer - 1'b1;
					if(!read_timer) begin
						write_data   <= 0;
						watchdog_set <= 0;
						s_drq_busy   <= 2'b11;
						state        <= STATE_WT_WAIT;
					end
				end
			STATE_WT_WAIT:
				begin
					if(watchdog_bark | (write_data & s_drq)) begin
						s_drq_busy <= 2'b01;
						if(watchdog_bark) s_lostdata <= 1;
						// a byte not given in time is written as 00, as the WD1793 does
						case(watchdog_bark ? 8'h00 : wdreg_data)
							8'hF5:
								begin
									wt_b1   <= 8'hA1;
									wt_n    <= 1;
									crc     <= crc_next(last_f5 ? crc : 16'hFFFF, 8'hA1);
									last_f5 <= 1;
								end
							8'hF6:
								begin
									wt_b1   <= 8'hC2;
									wt_n    <= 1;
									last_f5 <= 0;
								end
							8'hF7:
								begin
									wt_b1   <= crc[15:8];
									wt_b2   <= crc[7:0];
									wt_n    <= 2;
									last_f5 <= 0;
								end
							default:
								begin
									wt_b1   <= watchdog_bark ? 8'h00 : wdreg_data;
									wt_n    <= 1;
									crc     <= crc_next(crc, watchdog_bark ? 8'h00 : wdreg_data);
									last_f5 <= 0;
								end
						endcase
						state <= STATE_WT_EMIT;
					end
				end
			STATE_WT_EMIT:
				begin
					fsm_byte <= wt_b1;
					fsm_wr   <= 1;
					state    <= STATE_WT_ADV;
				end
			STATE_WT_ADV:
				begin
					fsm_wr   <= 0;
					wt_b1    <= wt_b2;
					wt_n     <= wt_n - 1'd1;
					tpos     <= tpos + 1'd1;
					scan_ptr <= scan_ptr + 1'd1;
					if(tpos == TRACK_LEN - 1) begin
						wt_end <= 1;
						state  <= STATE_WT_FLUSH;
					end
					else if(&scan_ptr) state <= STATE_WT_FLUSH;
					else state <= (wt_n == 2) ? STATE_WT_EMIT : STATE_WT_REQ;
				end
			STATE_WT_FLUSH:
				begin
					sd_busy <= 1;
					sd_wr   <= 1;
					state   <= STATE_WT_FLUSH_2;
				end
			STATE_WT_FLUSH_2:
				begin
					if(!sd_busy) begin
						if(wt_end || ({scan_lba + 1'd1, 9'd0} >= disk_size)) state <= STATE_ENDCOMMAND;
						else begin
							scan_lba <= scan_lba + 1'd1;
							load_ret <= STATE_WT_RESUME;
							state    <= STATE_LOAD_1;
						end
					end
				end

			// Abort current operation ($D0)
			STATE_ABORT:
				begin
					data_length <= 0;
					{s_wrfault,s_seekerr,s_crcerr,s_lostdata} <= 0;
					state <= STATE_ENDCOMMAND;
				end

			STATE_WAIT:
				begin
					wait_time <= 4000;
					state <= STATE_WAIT_2;
				end
			STATE_WAIT_2:
				begin
					if(wait_time) wait_time <= wait_time - 1;
						else state <= STATE_ENDCOMMAND;
				end

			// End any command.
			STATE_ENDCOMMAND:
				begin
					buff_rd <= 0;
					buff_wr <= 0;
					fsm_wr  <= 0;
					scan_mode <= 0;
					state <= STATE_IDLE;
					s_drq_busy <= 2'b00;
					seektimer <= 'h3FF;
					s_intrq <= 1;
				end
		endcase

		/* Register write operations */
		if (!old_wr & wre) begin
			case (addr)
				A_COMMAND:
					begin
						s_intrq <= 0;
						if((state == STATE_IDLE) | (din[7:4] == 'hD)) begin
							cmd_mode <= din[7];
							s_wpe    <= ~din[7];
							// Type I commands start with clean error bits
							if(~din[7]) {s_seekerr, s_crcerr} <= 0;
							case (din[7:4])
							'h0: 	// RESTORE
								begin
									// head load as specified, index, track0
									s_headloaded <= din[3];
									wdreg_track <= 0;
									disk_track <= 0;

									// some programs like it when FDC gets busy for a while
									s_drq_busy <= 2'b01;
									state <= STATE_WAIT;
								end
							'h1:	// SEEK
								begin
									// set real track to datareg
									disk_track <= wdreg_data;
									s_headloaded <= din[3];
									wdreg_track <= wdreg_data;
									// get busy
									s_drq_busy <= 2'b01;
									state <= STATE_WAIT;
								end
							'h2,	// STEP
							'h3,	// STEP & UPDATE
							'h4,	// STEP-IN
							'h5,	// STEP-IN & UPDATE
							'h6,	// STEP-OUT
							'h7:	// STEP-OUT & UPDATE
								begin
									// if direction is specified, store it for the next time
									if (din[6] == 1) step_direction <= din[5]; // 0: forward/in

									// perform step
									disk_track <= next_track;

									// update TRACK register too if asked to
									if (din[4]) wdreg_track <= next_track;

									s_headloaded <= din[3];

									// some programs like it when FDC gets busy for a while
									s_drq_busy <= 2'b01;
									state <= STATE_WAIT;
								end
							'h8, 'h9, // READ SECTORS
							'hA, 'hB: // WRITE SECTORS
								begin
									// 5: 0: read, 1: write
									// 4: m: 0: one sector, 1: until the track ends
									// 3: S: side to compare (not used)
									// 2: E: 15 ms delay
									// 1: C: side compare enable (not used)
									// 0: a0: deleted data mark on writes (not used)

									s_drq_busy <= 2'b01;
									{s_wrfault,s_seekerr,s_crcerr,s_lostdata} <= 0;

									{write,buff_rd} <= din[5] ? 2'b10 : 2'b01;
									buff_wr <= din[5];

									multisector <= din[4];
									wr_del      <= din[0];
									rw_type     <= 1;
									write_data  <= 0;
									read_data   <= 0;
									state       <= STATE_SEARCH;
									s_wpe       <= din[5];

									if(s_readonly & din[5]) begin
										s_wrfault <= 1;
										state <= STATE_WAIT;
									end
								end
							'hC:	// READ ADDRESS
								begin
									// track, side, sector, sector size code, 2-byte CRC
									s_drq_busy <= 2'b01;
									{s_wrfault,s_seekerr,s_crcerr,s_lostdata} <= 0;

									{write,buff_rd} <= 0;
									buff_wr <= 0;

									multisector <= 0;
									rw_type     <= 0;
									read_data   <= 0;
									data_length <= 6;
									state       <= STATE_SEARCH;
								end
							'hD:	// interrupt
								begin
									cmd_mode <= 0;
									if(state != STATE_IDLE) state <= STATE_ABORT;
										else {s_wrfault,s_seekerr,s_crcerr,s_lostdata, s_drq_busy} <= 0;
								end
							'hF:  // WRITE TRACK
								begin
									s_wpe <= 1;
									{s_wrfault,s_seekerr,s_crcerr,s_lostdata} <= 0;
									{write,buff_rd} <= 0;
									buff_wr     <= 0;
									s_drq_busy  <= 2'b01;
									write_data  <= 0;
									state       <= STATE_WT;
									if(s_readonly) begin
										s_wrfault <= 1;
										state <= STATE_WAIT;
									end
								end
							'hE:	// READ TRACK
								begin
									{s_wrfault,s_seekerr,s_crcerr,s_lostdata} <= 0;
									{write,buff_rd} <= 0;
									buff_wr    <= 0;
									s_drq_busy <= 2'b01;
									read_data  <= 0;
									state      <= STATE_RT;
								end
							endcase
						end
					end

				A_TRACK:  if (!s_busy) wdreg_track <= din;
				A_SECTOR: if (!s_busy) wdreg_sector <= din;
				A_DATA:   wdreg_data <= din;
			endcase
		end
	end

	// A new image invalidates the header and the track cache and ends any
	// command in progress, except one still waiting for the disk or for the
	// head to settle, which starts over on the new image
	old_mounted <= img_mounted;
	if(old_mounted && ~img_mounted) begin
		disk_size   <= |img_size[31:24] ? 24'hFFFFFF : img_size[23:0];
		hdr_loaded  <= 0;
		hdr_valid   <= 0;
		cache_valid <= 0;
		sec_idx     <= 0;
		if(state != STATE_IDLE && state != STATE_SEARCH && state != STATE_RT && state != STATE_WT) begin
			{sd_rd, sd_wr, sd_busy} <= 0;
			scan_mode <= 0;
			fsm_wr    <= 0;
			state     <= STATE_ENDCOMMAND;
		end
	end
end

function [7:0] mfm_sig(input [2:0] i);
	reg [63:0] s;
	begin
		s = "MFM_DISK";
		mfm_sig = s[63 - {i, 3'b000} -: 8];
	end
endfunction

endmodule

module wd1793_dpram #(parameter DATAWIDTH=8, ADDRWIDTH=11)
(
	input	                     clock,

	input	     [ADDRWIDTH-1:0] address_a,
	input	     [DATAWIDTH-1:0] data_a,
	input	                     wren_a,
	output reg [DATAWIDTH-1:0] q_a,

	input	     [ADDRWIDTH-1:0] address_b,
	input	     [DATAWIDTH-1:0] data_b,
	input	                     wren_b,
	output reg [DATAWIDTH-1:0] q_b
);

logic [DATAWIDTH-1:0] ram[0:(1<<ADDRWIDTH)-1];

always_ff@(posedge clock) begin
	if(wren_a) begin
		ram[address_a] <= data_a;
		q_a <= data_a;
	end else begin
		q_a <= ram[address_a];
	end
end

always_ff@(posedge clock) begin
	if(wren_b) begin
		ram[address_b] <= data_b;
		q_b <= data_b;
	end else begin
		q_b <= ram[address_b];
	end
end

endmodule
