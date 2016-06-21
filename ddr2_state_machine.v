`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 		 Intan Technologies, LLC
// 
// Design Name: 	 RHD2000 Rhythm Interface
// Module Name:    ddr2_state_machine
// Project Name:   Opal Kelly FPGA/USB RHD2000 Interface
// Target Devices: 
// Tool versions: 
// Description: 	 SDRAM memory controller read/write state machine
//                 Adapted from ddr2_test.v from Opal Kelly's RAMTester example
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////

//`default_nettype none

module ddr2_state_machine
	(
	input  wire          clk,
	input  wire          reset,
	input  wire          writes_en,
	input  wire          reads_en,
	input  wire          calib_done, 
	
	//DDR Input Buffer (ib_)
	output reg           ib_re,
	input  wire [31:0]   ib_data,
	input  wire [10:0]	 ib_count,
	input  wire          ib_valid,
	input  wire          ib_empty,
	
	//DDR Output Buffer (ob_)
	output reg           ob_we,
	output reg  [31:0]   ob_data,
	input  wire [10:0]	 ob_count,
	
	output reg           p0_rd_en_o, 
	input  wire          p0_rd_empty,
	input  wire [31:0]   p0_rd_data,
	
	input  wire          p0_cmd_full,
	output reg           p0_cmd_en,
	output reg  [2:0]    p0_cmd_instr,
	output reg  [29:0]   p0_cmd_byte_addr,
	output wire [5:0]    p0_cmd_bl_o, 
	input  wire          p0_wr_full,
	output reg           p0_wr_en,
	output reg  [31:0]   p0_wr_data,
	output wire [3:0]    p0_wr_mask,
	
	output reg [29:0]    cmd_byte_addr_wr,  // Added by Intan as output for capacity monitoring
	output reg [29:0]    cmd_byte_addr_rd,  // Added by Intan as output for capacity monitoring
    input wire [31:0]    ddr_burst_len,     // Added for usb3 to set burst length
    input wire           burst_override     // Assert to change burst_len
	);

	localparam FIFO_SIZE	  = 2048;
	//localparam BURST_LEN      = 32;  // Number of 32-bit data chunks transferred per cycle
									    // Must be multiple of two!
    // Note: This parameter was set to 32 in the Opal Kelly RAMTester example.
    // Intan set this parameter to its minimum allowable value (2) and see no
    // performance degradation.  With BURST_LEN set to 2, the FIFO accepts data
    // in 2*32 = 64-bit chunks, so it reads 64/16 = 4 16-bit words in every cycle.
    // Thus, it is important to ensure that every sampling time step transfers an
    // integer multiple of 4 words to the FIFO.  Otherwise, we will end up with 1-3
    // "leftover" data words that will not be accepted by the FIFO after sampling
    // has paused or stopped.
    //
    // Setting BURST_LEN to higher may result in faster input/output to the
    // SDRAM.

	reg [5:0]  burst_cnt;

	reg         write_mode;
	reg         read_mode;
	reg         reset_d;
    reg [31:0]  new_burst_len;      // Input gets pushed in here.
    reg [31:0]  active_burst_len;   // BURST_LEN should only change in idle state. Keeps active burst_len.


	assign p0_cmd_bl_o = active_burst_len - 1;
	assign p0_wr_mask = 4'b0000;

	always @(posedge clk) write_mode <= writes_en;
	always @(posedge clk) read_mode <= reads_en;
	always @(posedge clk) reset_d <= reset;

	integer state;
	localparam    s_idle  = 0,
				  s_write1 = 10,
				  s_write2 = 11,
				  s_write3 = 12,
				  s_read1 = 20,
				  s_read2 = 21,
				  s_read3 = 22,
				  s_read4 = 23;
				  
	always @(posedge clk) begin
		if (reset_d) begin
			state           <= s_idle;
			burst_cnt       <= 3'b000;
			cmd_byte_addr_wr  <= 0;
			cmd_byte_addr_rd  <= 0;
			p0_cmd_instr <= 3'b0;
			p0_cmd_byte_addr <= 30'b0;
            new_burst_len <= ddr_burst_len;    // on reset, saves input.
		end else begin
			p0_cmd_en  <= 1'b0;
			p0_wr_en <= 1'b0;
			ib_re <= 1'b0;
			p0_rd_en_o   <= 1'b0;
			ob_we <= 1'b0;
            new_burst_len <= burst_override ? 32'd2 : ddr_burst_len;   // Change to 2 for flushing.

			case (state)
                s_idle: begin
                    // Sync burst_len in idle state.
                    burst_cnt <= new_burst_len;
                    active_burst_len <= new_burst_len;
                    
                    // only start writing when initialization is done and
                    // there is sufficient data present in the input FIFO
                    if (calib_done==1 && write_mode==1 && (ib_count >= new_burst_len)) begin
                        state <= s_write1;
                    // Only start reading when init is done, there is room in
                    // the output FIFO buffer and (IMPORTANT, added this to
                    // ramtest code!) we are not reading past the point where
                    // data was just written.
                    end else if (calib_done==1 && read_mode==1 && (ob_count<(FIFO_SIZE-1-new_burst_len)) && (cmd_byte_addr_wr != cmd_byte_addr_rd)) begin
                        state <= s_read1;
                    end
                end

				s_write1: begin
					state <= s_write2;
					ib_re <= 1'b1;
				end

				s_write2: begin
					if(ib_valid==1) begin
						p0_wr_data <= ib_data;
						p0_wr_en   <= 1'b1;
						burst_cnt <= burst_cnt - 1;
						state <= s_write3;
					end
				end
				
				s_write3: begin
                    if (burst_cnt == 3'd0) begin
                        p0_cmd_en <= 1'b1;
                        p0_cmd_byte_addr <= cmd_byte_addr_wr;
                        cmd_byte_addr_wr <= cmd_byte_addr_wr + 4*active_burst_len;
                        p0_cmd_instr     <= 3'b000;
                        state <= s_idle;
                    end else begin
                        state <= s_write1;
                    end
                end

				s_read1: begin
					p0_cmd_byte_addr <= cmd_byte_addr_rd;
					cmd_byte_addr_rd <= cmd_byte_addr_rd + 4*active_burst_len;
					p0_cmd_instr     <= 3'b001;
					p0_cmd_en        <= 1'b1;
					state <= s_read2;
				end
				
				s_read2: begin
                    // if SDRAM not empty, read. otherwise wait.
					if(p0_rd_empty==0) begin
						p0_rd_en_o   <= 1'b1;
						state <= s_read3;
					end
				end
				
				s_read3: begin
					ob_data <= p0_rd_data;
                    ob_we <= 1'b1;
                    burst_cnt <= burst_cnt - 1;
                    state <= s_read4;
				end
				
				s_read4: begin
                    if (burst_cnt == 3'd0) begin
                        state <= s_idle;
                    end else begin
                        state <= s_read2;
                    end
                end
					
			endcase
		end
	end


endmodule
