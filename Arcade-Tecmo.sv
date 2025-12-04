//   __   __     __  __     __         __
//  /\ "-.\ \   /\ \/\ \   /\ \       /\ \
//  \ \ \-.  \  \ \ \_\ \  \ \ \____  \ \ \____
//   \ \_\\"\_\  \ \_____\  \ \_____\  \ \_____\
//    \/_/ \/_/   \/_____/   \/_____/   \/_____/
//   ______     ______       __     ______     ______     ______
//  /\  __ \   /\  == \     /\ \   /\  ___\   /\  ___\   /\__  _\
//  \ \ \/\ \  \ \  __<    _\_\ \  \ \  __\   \ \ \____  \/_/\ \/
//   \ \_____\  \ \_____\ /\_____\  \ \_____\  \ \_____\    \ \_\
//    \/_____/   \/_____/ \/_____/   \/_____/   \/_____/     \/_/
//
// https://joshbassett.info
// https://twitter.com/nullobject
// https://github.com/nullobject
//
// Copyright (c) 2020 Josh Bassett
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//LLAPI : llapi.sv needs to be in rtl folder and needs to be declared in file.qip (set_global_assignment -name SYSTEMVERILOG_FILE rtl/llapi.sv)

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [48:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler
	output        VGA_DISABLE, // analog out is off

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	input         CLK_AUDIO, // 24.576 MHz
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

assign VGA_F1 = 0;
assign FB_FORCE_BLANK = 0;
assign VGA_DISABLE = 0;

assign AUDIO_S   = 1;
assign AUDIO_R   = AUDIO_L;
assign AUDIO_MIX = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign LED_USER  = ioctl_download;

assign VIDEO_ARX = status[1] ? 8'd16 : status[2] ? 12'd2191 : 12'd2560;
assign VIDEO_ARY = status[1] ? 8'd9  : status[2] ? 12'd2560 : 12'd2191;

//LLAPI: OSD access combinaison
assign BUTTONS   = llapi_osd;
//END LLAPI

assign SDRAM_CLK = clk_sdram;

assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

//LLAPI
//assign USER_OUT = '1;
//END LLAPI

assign VGA_SCALER = 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;

`include "build_id.v"
localparam CONF_STR = {
  "A.Tecmo;;",
  //LLAPI Always ON menu indicator
  "-,<< LLAPI enabled >>;",
  "-,<< Use USER I/O port >>;",
  "-;",
  //END LLAPI	
  "O1,Aspect Ratio,Original,Wide;",
  "O2,Orientation,Horz,Vert;",
  "O3,Flip Screen,Off,On;",
  "O46,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
  "O7,Debug,Off,On;",
  "-;",
  "DIP;",
  "-;",
  "R0,Reset;",
  "J1,B0,B1,B2,1P Start,2P Start,Coin,Pause;",
  "jn,A,B,X,R,Select,L,Start;",
  "v,1;", // change version here because of changes to conf_str could break compatibility with older cfg files
  "V,v",`BUILD_DATE
};

////////////////////////////////////////////////////////////////////////////////
// CLOCKS
////////////////////////////////////////////////////////////////////////////////

wire clk_sys, clk_sdram;
wire locked;

pll pll
(
  .rst(RESET),
  .refclk(CLK_50M),
  .outclk_0(clk_sys),
  .outclk_1(clk_sdram),
  .locked(locked)
);

////////////////////////////////////////////////////////////////////////////////
// HPS IO
////////////////////////////////////////////////////////////////////////////////

wire  [1:0] buttons;
wire [31:0] status;
wire        forced_scandoubler;
wire [21:0] gamma_bus;
wire        direct_video;

wire [24:0] ioctl_addr;
wire  [7:0] ioctl_data;
wire        ioctl_wr;
wire        ioctl_download;
wire  [7:0] ioctl_index;

wire [10:0] ps2_key;

wire  [10:0] joystick_0, joystick_1;

//LLAPI : usb joy desintermediation
wire  [9:0] joystick_usb_0, joystick_usb_1;
//END LLAPI

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
  .clk_sys(clk_sys),
  .HPS_BUS(HPS_BUS),

  .buttons(buttons),
  .status(status),
  .status_menumask(direct_video),
  .forced_scandoubler(forced_scandoubler),
  .gamma_bus(gamma_bus),
  .direct_video(direct_video),
  .video_rotated(video_rotated),

  .ioctl_addr(ioctl_addr),
  .ioctl_dout(ioctl_data),
  .ioctl_wr(ioctl_wr),
  .ioctl_download(ioctl_download),
  .ioctl_index(ioctl_index),
  
  //LLAPI : usb joy desintermediation
  .joystick_0(joystick_usb_0),
  .joystick_1(joystick_usb_1),
  //END LLAPI 
  
  .ps2_key(ps2_key)
);

//////////////////   LLAPI   ///////////////////

reg llapi_button_pressed, llapi_button_pressed2;

always @(posedge CLK_50M) begin
        if (reset) begin
                llapi_button_pressed  <= 0;
                llapi_button_pressed2 <= 0;
        end else begin
                if (|llapi_buttons)
                        llapi_button_pressed  <= 1;
                if (|llapi_buttons2)
                        llapi_button_pressed2 <= 1;
        end
end

// controller id is 0 if there is either an Atari controller or no controller
// if id is 0, assume there is no controller until a button is pressed
// also check for 255 and treat that as 'no controller' as well
wire use_llapi  = llapi_en  && llapi_select && ((|llapi_type  && ~(&llapi_type))  || llapi_button_pressed);
wire use_llapi2 = llapi_en2 && llapi_select && ((|llapi_type2 && ~(&llapi_type2)) || llapi_button_pressed2);

wire [31:0] llapi_buttons, llapi_buttons2;
wire [71:0] llapi_analog, llapi_analog2;
wire [7:0]  llapi_type, llapi_type2;
wire llapi_en, llapi_en2;

wire llapi_select = 1'b1; //Always on, not based on OSD menu serial option

wire llapi_latch_o, llapi_latch_o2, llapi_data_o, llapi_data_o2;

//connection to USER_OUT port
// Indexes:
// 0 = D+    = P1 Latch
// 1 = D-    = P1 Data
// 2 = TX-   = LLAPI Enable
// 3 = GND_d = N/C
// 4 = RX+   = P2 Latch
// 5 = RX-   = P2 Data

always_comb begin
	USER_OUT = 6'b111111;
	if (llapi_select) begin
		USER_OUT[0] = llapi_latch_o;
		USER_OUT[1] = llapi_data_o;
		USER_OUT[2] = ~(llapi_select & ~OSD_STATUS);  // Blister LED
		USER_OUT[4] = llapi_latch_o2;
		USER_OUT[5] = llapi_data_o2;
	end
end

//Port 1 conf

LLAPI llapi
(
	.CLK_50M(CLK_50M),
	.LLAPI_SYNC(video_vs),
	.IO_LATCH_IN(USER_IN[0]),
	.IO_LATCH_OUT(llapi_latch_o),
	.IO_DATA_IN(USER_IN[1]),
	.IO_DATA_OUT(llapi_data_o),
	.ENABLE(llapi_select & ~OSD_STATUS),
	.LLAPI_BUTTONS(llapi_buttons),
	.LLAPI_ANALOG(llapi_analog),
	.LLAPI_TYPE(llapi_type),
	.LLAPI_EN(llapi_en)
);

//Port 2 conf

LLAPI llapi2
(
	.CLK_50M(CLK_50M),
	.LLAPI_SYNC(video_vs),
	.IO_LATCH_IN(USER_IN[4]),
	.IO_LATCH_OUT(llapi_latch_o2),
	.IO_DATA_IN(USER_IN[5]),
	.IO_DATA_OUT(llapi_data_o2),
	.ENABLE(llapi_select & ~OSD_STATUS),
	.LLAPI_BUTTONS(llapi_buttons2),
	.LLAPI_ANALOG(llapi_analog2),
	.LLAPI_TYPE(llapi_type2),
	.LLAPI_EN(llapi_en2)
);


//Controller string provided by core for reference (bits order is important)
//Controller specific mapping based on type. More info here : https://docs.google.com/document/d/12XpxrmKYx_jgfEPyw-O2zex1kTQZZ-NSBdLO2RQPRzM/edit
//llapi_Buttons [id] are HID [id - 1]

//  hps_io #(.CONF_STR(CONF_STR)) "J1,B0,B1,B2,Start P1, Start P2 ,Coin,Pause;",


//Port 1 mapping

wire [10:0] joy_ll_a;
always_comb begin
	// button layout for 6 button controllers
	if (llapi_type == 20 || llapi_type == 21 || llapi_type == 8 || llapi_type == 3 || llapi_type == 11) begin
		joy_ll_a = {
			1'b0, // Pause
			llapi_buttons[4], 1'b0, llapi_buttons[5], // Coin, Start P2, Start P1
			llapi_buttons[7], llapi_buttons[1], llapi_buttons[0], // B2, B1, B0
			llapi_buttons[27], llapi_buttons[26], llapi_buttons[25], llapi_buttons[24] // d-pad
		};
	// button layout for NEC Avenue 6 Pad
	end else if (llapi_type == 54) begin
		joy_ll_a = {
			1'b0, // Pause
			llapi_buttons[4], 1'b0, llapi_buttons[5], // Coin, Start P2, Start P1
			llapi_buttons[1], llapi_buttons[0], llapi_buttons[7], // B2, B1, B0
			llapi_buttons[27], llapi_buttons[26], llapi_buttons[25], llapi_buttons[24] // d-pad
		};
	end else begin
		joy_ll_a = {
			1'b0, // Pause
			llapi_buttons[4], 1'b0, llapi_buttons[5], // Coin, Start P2, Start P1
			llapi_buttons[1], llapi_buttons[0], llapi_buttons[2], // B2, B1, B0
			llapi_buttons[27], llapi_buttons[26], llapi_buttons[25], llapi_buttons[24] // d-pad
		};
	end
end


//Port 2 mapping

wire [10:0] joy_ll_b;
always_comb begin
	// button layout for 6 button controllers
	if (llapi_type2 == 20 || llapi_type2 == 21 || llapi_type2 == 8 || llapi_type2 == 3 || llapi_type2 == 11) begin
		joy_ll_b = {
			1'b0, // Pause
			llapi_buttons2[4], llapi_buttons2[5], 1'b0, // Coin, Start P2, Start P1
			llapi_buttons2[7], llapi_buttons2[1], llapi_buttons2[0], // B2, B1, B0
			llapi_buttons2[27], llapi_buttons2[26], llapi_buttons2[25], llapi_buttons2[24] // d-pad
		};
	// button layout for NEC Avenue 6 Pad
	end else if (llapi_type2 == 54) begin
		joy_ll_b = {
			1'b0, // Pause
			llapi_buttons2[4], llapi_buttons2[5],1'b0, // Coin, Start P2, Start P1
			llapi_buttons2[1], llapi_buttons2[0], llapi_buttons2[7], // B2, B1, B0
			llapi_buttons2[27], llapi_buttons2[26], llapi_buttons2[25], llapi_buttons2[24] // d-pad
		};
	end else begin
		joy_ll_b = {
			1'b0, // Pause
			llapi_buttons2[4], llapi_buttons2[5], 1'b0, // Coin, Start P2, Start P1
			llapi_buttons2[1], llapi_buttons2[0], llapi_buttons2[2], // B2, B1, B0
			llapi_buttons2[27], llapi_buttons2[26], llapi_buttons2[25], llapi_buttons2[24] // d-pad
		};
	end
end

//Assign (DOWN + START + FIRST BUTTON) Combinaison to bring the OSD up - P1 and P2 ports.
wire llapi_osd = (llapi_buttons[26] & llapi_buttons[5] & llapi_buttons[0]) || (llapi_buttons2[26] & llapi_buttons2[5] & llapi_buttons2[0]);

assign joystick_0 = joy_ll_a;
assign joystick_1 = joy_ll_b;


//////////////////  END LLAPI  ///////////////////




////////////////////////////////////////////////////////////////////////////////
// VIDEO
////////////////////////////////////////////////////////////////////////////////

wire       ce_pix;
wire [3:0] r, g, b;
wire       hsync, vsync;
wire       hblank, vblank;
wire       no_rotate = ~status[2] & ~direct_video;
wire flip = 0;
wire video_rotated;
wire rotate_ccw = 1'b0;

arcade_video #(.WIDTH(256), .DW(12)) arcade_video
(
  .*,
  .clk_video(clk_sys),
  .ce_pix(ce_pix),
  .RGB_in({r, g, b}),
  .HBlank(hblank),
  .VBlank(vblank),
  .HSync(hsync),
  .VSync(vsync),
  .fx(status[6:4])
);

screen_rotate screen_rotate (.*);

////////////////////////////////////////////////////////////////////////////////
// SDRAM
////////////////////////////////////////////////////////////////////////////////

wire [22:0] sdram_addr;
wire [31:0] sdram_data;
wire        sdram_we;
wire        sdram_req;
wire        sdram_ack;
wire        sdram_valid;
wire [31:0] sdram_q;

sdram #(.CLK_FREQ(96.0)) sdram
(
  .reset(~locked),
  .clk(clk_sys),

  // controller interface
  .addr(sdram_addr),
  .data(sdram_data),
  .we(sdram_we),
  .req(sdram_req),
  .ack(sdram_ack),
  .valid(sdram_valid),
  .q(sdram_q),

  // SDRAM interface
  .sdram_a(SDRAM_A),
  .sdram_ba(SDRAM_BA),
  .sdram_dq(SDRAM_DQ),
  .sdram_cke(SDRAM_CKE),
  .sdram_cs_n(SDRAM_nCS),
  .sdram_ras_n(SDRAM_nRAS),
  .sdram_cas_n(SDRAM_nCAS),
  .sdram_we_n(SDRAM_nWE),
  .sdram_dqml(SDRAM_DQML),
  .sdram_dqmh(SDRAM_DQMH)
);

////////////////////////////////////////////////////////////////////////////////
// CONTROLS
////////////////////////////////////////////////////////////////////////////////

wire       pressed = ps2_key[9];
wire [7:0] code    = ps2_key[7:0];

reg key_left  = 0;
reg key_right = 0;
reg key_down  = 0;
reg key_up    = 0;
reg key_ctrl  = 0;
reg key_alt   = 0;
reg key_space = 0;
reg key_1     = 0;
reg key_2     = 0;
reg key_5     = 0;
reg key_6     = 0;
reg key_a     = 0;
reg key_s     = 0;
reg key_q     = 0;
reg key_r     = 0;
reg key_f     = 0;
reg key_d     = 0;
reg key_g     = 0;
reg key_p     = 0;

always @(posedge clk_sys) begin
  reg old_state;
  old_state <= ps2_key[10];

  if (old_state != ps2_key[10]) begin
    case (code)
      'h75: key_up    <= pressed;
      'h72: key_down  <= pressed;
      'h6B: key_left  <= pressed;
      'h74: key_right <= pressed;
      'h14: key_ctrl  <= pressed;
      'h11: key_alt   <= pressed;
      'h29: key_space <= pressed;
      'h16: key_1     <= pressed;
      'h1E: key_2     <= pressed;
      'h2E: key_5     <= pressed;
      'h36: key_6     <= pressed;
      'h1C: key_a     <= pressed;
      'h1B: key_s     <= pressed;
      'h15: key_q     <= pressed;
      'h2D: key_r     <= pressed;
      'h2B: key_f     <= pressed;
      'h23: key_d     <= pressed;
      'h34: key_g     <= pressed;
      'h4d: key_p     <= pressed;
    endcase
  end
end

wire player_1_up       = key_up    | joystick_0[3];
wire player_1_down     = key_down  | joystick_0[2];
wire player_1_left     = key_left  | joystick_0[1];
wire player_1_right    = key_right | joystick_0[0];
wire player_1_button_1 = key_ctrl  | joystick_0[4];
wire player_1_button_2 = key_alt   | joystick_0[5];
wire player_1_button_3 = key_space | joystick_0[6];
wire player_1_start    = key_1     | joystick_0[7] | joystick_1[7];
wire player_1_coin     = key_5     | joystick_0[9];
wire player_1_pause    = key_p     | joystick_0[10];
wire player_2_up       = key_r     | joystick_1[3];
wire player_2_down     = key_f     | joystick_1[2];
wire player_2_left     = key_d     | joystick_1[1];
wire player_2_right    = key_g     | joystick_1[0];
wire player_2_button_1 = key_a     | joystick_1[4];
wire player_2_button_2 = key_s     | joystick_1[5];
wire player_2_button_3 = key_q     | joystick_1[6];
wire player_2_start    = key_2     | joystick_1[8] | joystick_0[8];
wire player_2_coin     = key_6     | joystick_1[9];
wire player_2_pause    =             joystick_1[10];

////////////////////////////////////////////////////////////////////////////////
// GAME
////////////////////////////////////////////////////////////////////////////////

wire reset = RESET | status[0] | buttons[1] | ~locked;
reg [7:0] sw[8];
reg [3:0] game_index = 0;

always @(posedge clk_sys) begin
  // set game index
  if (ioctl_wr && (ioctl_index == 1)) game_index <= ioctl_data[3:0];

  // set DIP switches
  if (ioctl_wr && (ioctl_index == 254) && !ioctl_addr[24:3]) sw[ioctl_addr[2:0]] <= ioctl_data;
end

tecmo #(.CLK_FREQ(96.0)) tecmo
(
  .reset(reset),

  .clk(clk_sys),
  .cen_6(ce_pix),

  .debug(status[7]),
  .flip(status[3]),

  .joy_1({player_1_up, player_1_down, player_1_right, player_1_left}),
  .joy_2({player_2_up, player_2_down, player_2_right, player_2_left}),
  .buttons_1({2'b0, player_1_button_3, player_1_button_2, player_1_button_1}),
  .buttons_2({2'b0, player_2_button_3, player_2_button_2, player_2_button_1}),
  .sys({player_1_coin, player_2_coin, player_1_start, player_2_start}),
  .pause(player_1_pause | player_2_pause),

  .dip_1(sw[0]),
  .dip_2(sw[1]),

  .sdram_addr(sdram_addr),
  .sdram_data(sdram_data),
  .sdram_we(sdram_we),
  .sdram_req(sdram_req),
  .sdram_ack(sdram_ack),
  .sdram_valid(sdram_valid),
  .sdram_q(sdram_q),

  .ioctl_addr(ioctl_addr[19:0]),
  .ioctl_data(ioctl_data),
  .ioctl_wr(ioctl_wr && !ioctl_index),
  .ioctl_download(ioctl_download && !ioctl_index),

  .game_index(game_index),

  .hsync(hsync),
  .vsync(vsync),
  .hblank(hblank),
  .vblank(vblank),

  .r(r),
  .g(g),
  .b(b),

  .audio(AUDIO_L)
);

endmodule