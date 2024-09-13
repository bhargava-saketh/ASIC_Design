
`timescale 1ns / 1ps
module PISO (Sclk, Clear_sig, Frame_sig, ShiftComplete_sig, SerialOut_sig, p2s_en_sig, OutReady_sig);
input Sclk, Clear_sig, p2s_en_sig, Frame_sig;
input [39:0] ShiftComplete_sig;
output reg SerialOut_sig, OutReady_sig;
reg [5:0] bit_counter_reg;

reg is_ready_out_reg, frame_active_reg;
reg [39:0] piso_register_reg;
	always @(posedge Sclk)
	begin
		if(Clear_sig == 1'b1)
		begin
			bit_counter_reg = 6'd40;
			piso_register_reg = 40'd0;
			is_ready_out_reg = 1'b0;
			frame_active_reg = 1'b0;
			OutReady_sig = 1'b0;
			SerialOut_sig = 1'b0;
		end
		else if (p2s_en_sig == 1'b1)
		begin
			piso_register_reg = ShiftComplete_sig;
			is_ready_out_reg = 1'b1;
		end
		else if (Frame_sig == 1'b1 && is_ready_out_reg == 1'b1 && frame_active_reg == 1'b0)
		begin
			bit_counter_reg = bit_counter_reg - 1'b1;
			SerialOut_sig = piso_register_reg [bit_counter_reg];
			frame_active_reg = 1'b1;
			is_ready_out_reg = 1'b0;
			OutReady_sig = 1'b1;
		end
		else if (frame_active_reg == 1'b1)
		begin
			bit_counter_reg = bit_counter_reg - 1'b1;
			SerialOut_sig = piso_register_reg [bit_counter_reg];
			OutReady_sig = 1'b1;
			if (bit_counter_reg == 6'd0)
				frame_active_reg = 1'b0;
		end
		else
		begin
			bit_counter_reg = 6'd40;
			SerialOut_sig = 1'b0;
			OutReady_sig = 1'b0;
		end
	end
endmodule

module SIPO (Frame_sig, Dclk, Clear_sig, InputLeft_sig, InputRight_sig, dataL_reg, dataR_reg, in_flag_sig);
	input Frame_sig, Dclk, Clear_sig, InputLeft_sig, InputRight_sig;
	output reg in_flag_sig;
	output reg [15:0] dataL_reg;
	output reg [15:0] dataR_reg;
	reg [3:0] count_bit_reg;
	reg frame_stat_reg;

	always @(negedge Dclk or posedge Clear_sig)
	begin
		if (Clear_sig == 1'b1)
		begin
			count_bit_reg = 4'd15;
			dataL_reg = 16'd0;
			dataR_reg = 16'd0;			
			in_flag_sig = 1'b0;
			frame_stat_reg = 1'b0;
		end
		else 
		begin
			if (Frame_sig == 1'b1)
			begin
				count_bit_reg = 4'd15;
				in_flag_sig = 1'b0;
				dataL_reg [count_bit_reg] = InputLeft_sig;
				dataR_reg [count_bit_reg] = InputRight_sig;
				frame_stat_reg = 1'b1;
			end
			else if (frame_stat_reg == 1'b1)
			begin
				count_bit_reg = count_bit_reg - 1'b1;
				dataL_reg [count_bit_reg] = InputLeft_sig;
				dataR_reg [count_bit_reg] = InputRight_sig;
				if (count_bit_reg == 4'd0)
				begin					
					in_flag_sig = 1'b1;
					frame_stat_reg = 1'b0;
				end
				else
				begin
					in_flag_sig = 1'b0;
					frame_stat_reg = 1'b1;
				end
			end
			else
			begin
				count_bit_reg = 4'd15;
				dataL_reg = 16'd0;
				dataR_reg = 16'd0;			
				in_flag_sig = 1'b0;
				frame_stat_reg = 1'b0;
			end
		end
	end
endmodule

module adder(
	input [39:0] a_sig,
    input [39:0] b_sig,
    input addsub_sig,
	input adder_en_sig,
    output [39:0] sum_sig
    );

		assign sum_sig = (addsub_sig == 1'b1) ? (b_sig - a_sig) : 
						 (addsub_sig == 1'b0) ? (b_sig + a_sig) :
						 	sum_sig;	

endmodule

module alu_controller (
	input work_enable_sig,
	input Clear_sig,
	input Sclk,
	input sleep_flag_sig,
	
	input [7:0] rjdataL_sig, 
input [8:0] coeffdataL_sig, 
input [15:0] indataL_sig,
	input [7:0] rjdataR_sig, 
input [8:0] coeffdataR_sig, 
input [15:0] indataR_sig,
	
	output [39:0] addL_input_sig, addR_input_sig,
	output reg rjL_enable_sig, coeffL_enable_sig, inputL_enable_sig,
	output reg rjR_enable_sig, coeffR_enable_sig, inputR_enable_sig,
	output reg addsubL_sig, adderL_en_sig, shiftL_en_sig, loadL_sig, clearL_sig, p2sL_en_sig,
	output reg addsubR_sig, adderR_en_sig, shiftR_en_sig, loadR_sig, clearR_sig, p2sR_en_sig,
    output reg [3:0] rjL_addr_reg,
	output reg [8:0] coeffL_addr_reg,
	output reg [7:0] inputL_addr_reg,
	
	output reg [3:0] rjR_addr_reg,
	output reg [8:0] coeffR_addr_reg,
	output reg [7:0] inputR_addr_reg
	);
	
	parameter start_ini_state = 2'b00, work_state = 2'b01, sleep_state = 2'b10;
	
	reg memL_overflow_reg, start_workL_reg, work_statusL_reg, outL_reg;
	reg memR_overflow_reg, start_workR_reg, work_statusR_reg, outR_reg;
	reg [1:0] present_stateL_reg, next_stateL_reg;
	reg [1:0] present_stateR_reg, next_stateR_reg;
	reg [7:0] kL_reg, kR_reg;
	reg [7:0] xcntL_reg, x_indexL_reg;
	reg [7:0] xcntR_reg, x_indexR_reg;
	
	assign addL_input_sig = (indataL_sig[15]) ? {8'hFF, indataL_sig, 16'h0000} : {8'h00, indataL_sig, 16'h0000};
	assign addR_input_sig = (indataR_sig[15]) ? {8'hFF, indataR_sig, 16'h0000} : {8'h00, indataR_sig, 16'h0000};
	
		
	always @(Clear_sig, next_stateL_reg)
	begin
		if (Clear_sig == 1'b1)
			present_stateL_reg <= start_ini_state;
		else
			present_stateL_reg <= next_stateL_reg;
	end
	
	always @(negedge Sclk)
	begin
		case (present_stateL_reg)
			start_ini_state:
				begin
					memL_overflow_reg <= 1'b0;
					if (Clear_sig == 1'b1)
						next_stateL_reg <= start_ini_state;
					else if (work_enable_sig == 1'b1)
					begin
						next_stateL_reg <= work_state;
						xcntL_reg <= 8'd1;
						start_workL_reg <= 1'b1;
						work_statusL_reg <= 1'b1;
					end
					else
					begin
						next_stateL_reg <= start_ini_state;
						xcntL_reg <= xcntL_reg;
						start_workL_reg <= 1'b0;
					end
				end
			
			work_state:
				begin
					if (work_enable_sig == 1'b1)
					begin
						xcntL_reg <= xcntL_reg + 1'b1;
						start_workL_reg <= 1'b1;
						work_statusL_reg <= 1'b1;
						if (xcntL_reg == 8'hFF)
							memL_overflow_reg <= 1'b1;
						else
							memL_overflow_reg <= memL_overflow_reg;
					end
					else
					begin
						start_workL_reg <= 1'b0;
						memL_overflow_reg <= memL_overflow_reg;
						if (rjL_addr_reg == 4'h8 && coeffL_addr_reg == 9'h1FF && kL_reg == rjdataL_sig)
							work_statusL_reg <= 1'b0;
						else
							work_statusL_reg <= work_statusL_reg;
					end
					
					if (Clear_sig == 1'b1)
						next_stateL_reg <= start_ini_state;
					else
						next_stateL_reg <= work_state;
				end
			
			sleep_state:
				begin
					xcntL_reg <= xcntL_reg;
					memL_overflow_reg <= memL_overflow_reg;
					start_workL_reg <= 1'b0;
					work_statusL_reg <= 1'b0;
					if (Clear_sig == 1'b1)
						next_stateL_reg <= start_ini_state;
					else if (sleep_flag_sig == 1'b0)
					begin
						xcntL_reg <= xcntL_reg + 1'b1;
						start_workL_reg <= 1'b1;
						work_statusL_reg <= 1'b1;
						if (xcntL_reg == 8'hFF)
							memL_overflow_reg <= 1'b1;
						else
							memL_overflow_reg <= memL_overflow_reg;
						next_stateL_reg <= work_state;
					end
					else
						next_stateL_reg <= sleep_state;
				end
				
			default:	next_stateL_reg <= start_ini_state;
		endcase
	end
	
	always @(negedge Sclk)
	begin
		if (outL_reg)
		begin
			p2sL_en_sig = 1'b1;
			rjL_addr_reg = 4'd0;
			coeffL_addr_reg = 9'd0;
			kL_reg = 8'd0;
			outL_reg = 1'b0;
			clearL_sig = 1'b1;
		end
		else
			p2sL_en_sig = 1'b0;
		
		if (start_workL_reg == 1'b1)
		begin
			outL_reg = 1'b0;
			rjL_addr_reg = 4'd0;
			rjL_enable_sig = 1'b1;
			coeffL_addr_reg = 9'd0;
			coeffL_enable_sig = 1'b1;
			inputL_enable_sig = 1'b0;
			adderL_en_sig = 1'b0;
			shiftL_en_sig = 1'b0;
			kL_reg = 8'd0;
			clearL_sig = 1'b1;
			loadL_sig = 1'b0;
		end
		else if (work_statusL_reg == 1'b1)
		begin
			//$display("KL_reg", $time, kL_reg);
			//$display("rjdataL_sig", $time, rjdataL_sig);
			if (kL_reg == rjdataL_sig)
			begin
				inputL_enable_sig = 1'b0;
				shiftL_en_sig = 1'b1;
				clearL_sig = 1'b0;
				loadL_sig = 1'b1;
				adderL_en_sig = 1'b1;
				kL_reg = 8'd0;
				if (rjL_addr_reg < 4'd8)
				begin
					rjL_addr_reg = rjL_addr_reg + 1'b1;
				end
				else
				begin
					rjL_addr_reg = 4'd0;
					outL_reg = 1'b1;
					coeffL_addr_reg = 9'd0;
				end
			end
			else
			begin
				shiftL_en_sig = 1'b0;
				clearL_sig = 1'b0;
				loadL_sig = 1'b0;
				inputL_enable_sig = 1'b0;
				x_indexL_reg = coeffdataL_sig[7:0];
				addsubL_sig = coeffdataL_sig[8];
				if (xcntL_reg - 1'b1 >= x_indexL_reg)
				begin
					inputL_addr_reg = xcntL_reg - 1'b1 - x_indexL_reg;
					inputL_enable_sig = 1'b1;
					adderL_en_sig = 1'b1;
					loadL_sig = 1'b1;
				end
				else if (xcntL_reg - 1'b1 < x_indexL_reg && memL_overflow_reg == 1'b1)
				begin
					inputL_addr_reg = xcntL_reg - 1'b1 + (9'd256 - x_indexL_reg);
					inputL_enable_sig = 1'b1;
					adderL_en_sig = 1'b1;
					loadL_sig = 1'b1;
				end
				else
				begin
					inputL_addr_reg = inputL_addr_reg;
					adderL_en_sig = 1'b0;
				end
				
				if (coeffL_addr_reg < 9'h1FF)
					coeffL_addr_reg = coeffL_addr_reg + 1'b1;
				else
					coeffL_addr_reg = coeffL_addr_reg;
				
				kL_reg = kL_reg + 1'b1;
			end
		end
		else
		begin
			rjL_addr_reg = 4'd0;
			rjL_enable_sig = 1'b0;
			coeffL_addr_reg = 9'd0;
			coeffL_enable_sig = 1'b0;
			inputL_enable_sig = 1'b0;
			adderL_en_sig = 1'b0;
			shiftL_en_sig = 1'b0;
			kL_reg = 8'd0;
			loadL_sig = 1'b0;
			clearL_sig = 1'b1;
		end
	end
	
	
	// Right side FSM
	
	always @(Clear_sig, next_stateR_reg)
	begin
		if (Clear_sig == 1'b1)
			present_stateR_reg <= start_ini_state;
		else
			present_stateR_reg <= next_stateR_reg;
	end
	
	always @(negedge Sclk)
	begin
		case (present_stateR_reg)
			start_ini_state:
				begin
					memR_overflow_reg <= 1'b0;
					if (Clear_sig == 1'b1)
						next_stateR_reg <= start_ini_state;
					else if (work_enable_sig == 1'b1)
					begin
						next_stateR_reg <= work_state;
						xcntR_reg <= 8'd1;
						start_workR_reg <= 1'b1;
						work_statusR_reg <= 1'b1;
					end
					else
					begin
						next_stateR_reg <= start_ini_state;
						xcntR_reg <= xcntR_reg;
						start_workR_reg <= 1'b0;
					end
				end
			
			work_state:
				begin
					if (work_enable_sig == 1'b1)
					begin
						xcntR_reg <= xcntR_reg + 1'b1;
						start_workR_reg <= 1'b1;
						work_statusR_reg <= 1'b1;
						if (xcntR_reg == 8'hFF)
							memR_overflow_reg <= 1'b1;
						else
							memR_overflow_reg <= memR_overflow_reg;
					end
					else
					begin
						start_workR_reg <= 1'b0;
						memR_overflow_reg <= memR_overflow_reg;
						if (rjR_addr_reg == 4'hb && coeffR_addr_reg == 9'h1FF && kR_reg == rjdataR_sig)
							work_statusR_reg <= 1'b0;
						else
							work_statusR_reg <= work_statusR_reg;
					end
					
					if (Clear_sig == 1'b1)
						next_stateR_reg <= start_ini_state;
					else
						next_stateR_reg <= work_state;
				end
			
			sleep_state:
				begin
					xcntR_reg <= xcntR_reg;
					memR_overflow_reg <= memR_overflow_reg;
					start_workR_reg <= 1'b0;
					work_statusR_reg <= 1'b0;
					if (Clear_sig == 1'b1)
						next_stateR_reg <= start_ini_state;
					else if (sleep_flag_sig == 1'b0)
					begin
						xcntR_reg <= xcntR_reg + 1'b1;
						start_workR_reg <= 1'b1;
						work_statusR_reg <= 1'b1;
						if (xcntR_reg == 8'hFF)
							memR_overflow_reg <= 1'b1;
						else
							memR_overflow_reg <= memR_overflow_reg;
						next_stateR_reg <= work_state;
					end
					else
						next_stateR_reg <= sleep_state;
				end
				
			default:
				begin
				end
		endcase
	end
	
	always @(negedge Sclk)
	begin
		if (outR_reg)
		begin
			p2sR_en_sig = 1'b1;
			rjR_addr_reg = 4'd0;
			coeffR_addr_reg = 9'd0;
			kR_reg = 8'd0;
			outR_reg = 1'b0;
		end
		else
			p2sR_en_sig = 1'b0;
		
		if (start_workR_reg == 1'b1)
		begin
			outR_reg = 1'b0;
			rjR_addr_reg = 4'd0;
			rjR_enable_sig = 1'b1;
			coeffR_addr_reg = 9'd0;
			coeffR_enable_sig = 1'b1;
			inputR_enable_sig = 1'b0;
			adderR_en_sig = 1'b0;
			shiftR_en_sig = 1'b0;
			kR_reg = 8'd0;
			clearR_sig = 1'b1;
			loadR_sig = 1'b0;
		end
		else if (work_statusR_reg == 1'b1)
		begin
			//$display("KR_reg", $time, kR_reg);
			//$display("rjdataR_sig", $time, rjdataR_sig);
				if (kR_reg == rjdataR_sig)
			begin
				inputR_enable_sig = 1'b0;
				shiftR_en_sig = 1'b1;
				clearR_sig = 1'b0;
				loadR_sig = 1'b1;
				adderR_en_sig = 1'b1;
				kR_reg = 8'd0;
				if (rjR_addr_reg < 4'd11)
				begin
					rjR_addr_reg = rjR_addr_reg + 1'b1;
				end
				else
				begin
					rjR_addr_reg = 4'd0;
					outR_reg = 1'b1;
					coeffR_addr_reg = 9'd0;
				end
			end
			else
			begin
				shiftR_en_sig = 1'b0;
				clearR_sig = 1'b0;
				loadR_sig = 1'b0;
				inputR_enable_sig = 1'b0;
				x_indexR_reg = coeffdataR_sig[7:0];
				addsubR_sig = coeffdataR_sig[8];
				if (xcntR_reg - 1'b1 >= x_indexR_reg)
				begin
					inputR_addr_reg = xcntR_reg - 1'b1 - x_indexR_reg;
					inputR_enable_sig = 1'b1;
					adderR_en_sig = 1'b1;
					loadR_sig = 1'b1;
				end
				else if (xcntR_reg - 1'b1 < x_indexR_reg && memR_overflow_reg == 1'b1)
				begin
					inputR_addr_reg = xcntR_reg - 1'b1 + (9'd256 - x_indexR_reg);
					inputR_enable_sig = 1'b1;
					adderR_en_sig = 1'b1;
					loadR_sig = 1'b1;
				end
				else
				begin
					inputR_addr_reg = inputR_addr_reg;
					adderR_en_sig = 1'b0;
				end
				
				if (coeffR_addr_reg < 9'h1FF)
					coeffR_addr_reg = coeffR_addr_reg + 1'b1;
				else
					coeffR_addr_reg = coeffR_addr_reg;
				
				kR_reg = kR_reg + 1'b1;
			end
		end
		else
		begin
			rjR_addr_reg = 4'd0;
			rjR_enable_sig = 1'b0;
			coeffR_addr_reg = 9'd0;
			coeffR_enable_sig = 1'b0;
			inputR_enable_sig = 1'b0;
			adderR_en_sig = 1'b0;
			shiftR_en_sig = 1'b0;
			kR_reg = 8'd0;
			loadR_sig = 1'b0;
			clearR_sig = 1'b1;
		end
	end
	
endmodule
/*
 module coeff_memory (input write_enable_sig, read_enable_sig, Sclk,
							input [15:0] in_data,
input [8:0] coeffwrite, coeffread,							
							output [15:0] data_coeff_reg);
	reg [15:0] coeffmem [0:511];

	always @(posedge Sclk)
	begin
		if(write_enable_sig == 1'b1)
			coeffmem[coeffwrite] = in_data;
		else
			coeffmem[coeffwrite] = coeffmem[coeffwrite];		
	end

	assign data_coeff_reg = (read_enable_sig) ? coeffmem[coeffread] : 16'd0;
endmodule

module data_memory (input write_enable_sig, read_enable_sig, Sclk, in_flag_sig,
					input [15:0] in_data,
input [7:0] datawrite, dataread,
					output [15:0] input_data_reg,
					output reg flag_zero_reg);

	reg [15:0] datamem [0:255];
	reg [11:0] count_zero_reg;
	reg [11:0] ani_cnt_reg;
	
	always @(posedge Sclk)
	begin
		if(write_enable_sig == 1'b1) begin
			ani_cnt_reg = ani_cnt_reg + 1'b1;
			datamem[datawrite] = in_data; end
		else
			datamem[datawrite] = datamem[datawrite];
	end
	always @(posedge in_flag_sig)
	begin
		if (in_data == 16'd0)
		begin
			count_zero_reg = count_zero_reg + 1'b1;
			if (count_zero_reg == 10'd800)
				flag_zero_reg = 1'b1;
			else if (count_zero_reg > 10'd800)
			begin
				count_zero_reg = 10'd800;
				flag_zero_reg = 1'b1;
			end
		end		
		else if (in_data != 16'd0)
		begin
			count_zero_reg = 12'd0;
			flag_zero_reg = 1'b0;
		end
	end

	assign input_data_reg = (read_enable_sig) ? datamem[dataread] : 16'd0;
endmodule 
*/
module main_controller (input Sclk, Dclk, Start_sig, Reset_n_sig, Frame_sig, in_flag_sig, flag_zeroL_sig, flag_zeroR_sig,
								 output reg [3:0] rjwrite_reg,
						       output reg [8:0] coeffwrite_reg,
						       output reg [7:0] datawrite_reg,
						       output reg rj_enable_sig, coeff_enable_sig, data_enable_sig, Clear_sig,
						       output Frame_out_sig, Dclk_out_sig, Sclk_out_sig,
						       output reg work_enable_sig, sleep_flag_sig, InReady_sig);
	
	parameter [3:0] Start_ini = 4'd0, Wait_rj = 4'd1, Read_rj = 4'd2,
					    Wait_coeff = 4'd3, Read_coeff = 4'd4, Wait_input = 4'd5,
					    Working = 4'd6, Reset = 4'd7, Sleep = 4'd8;
   
	reg [3:0] state_present_reg, state_next_reg;
	reg [15:0] count_reg;
	reg [15:0] addr = 0;

	reg [4:0] count_rj_reg;
	reg [9:0] count_coeff_reg;
	reg [7:0] count_data_reg;
	
	reg [15:0] rjmem [0:15];
	reg [15:0] coeffmem [0:511];
	reg [15:0] datamem [0:255];
	
	
	reg tk_reg;
	
	assign Frame_out_sig = Frame_sig;
	assign Dclk_out_sig = Dclk;
	assign Sclk_out_sig = Sclk;
	
	always @(posedge Sclk or negedge Reset_n_sig)		// Sequential block
	begin
		if (!Reset_n_sig)
		begin
			if (state_present_reg > 4'd4)
				state_present_reg = Reset;
			else
				state_present_reg = state_next_reg;
		end
		else
		state_present_reg = state_next_reg;
	end
	
	always @(posedge Sclk or posedge Start_sig)
	begin
		if (Start_sig == 1'b1) begin
			state_next_reg = Start_ini;
			addr <= 0; end
		else
		begin
		case (state_present_reg)
			Start_ini:	begin
							if(addr < 512) begin
							rjmem[addr[3:0]] <= 4'd0;
							coeffmem[addr[8:0]] <= 9'd0;
							datamem[addr[7:0]] <= 8'd0;
							if(addr < 512) begin
								addr <= addr + 1; end
							end
							else begin
							rjwrite_reg = 4'd0;
							coeffwrite_reg = 9'd0;
							datawrite_reg = 8'd0;
							rj_enable_sig = 1'b0;
							coeff_enable_sig = 1'b0;
							data_enable_sig = 1'b0;
							Clear_sig = 1'b1;
							work_enable_sig = 1'b0;
							InReady_sig = 1'b0;
							sleep_flag_sig = 1'b0;
							count_reg = 16'd0;
							count_rj_reg = 4'd0;
							count_coeff_reg = 9'd0;
							count_data_reg = 8'd0; 						
							state_next_reg = Wait_rj; 
							addr <= 0;
							end
					end
			
			Wait_rj:	begin
							rjwrite_reg = 4'd0;
							coeffwrite_reg = 9'd0;
							datawrite_reg = 8'd0;
							rj_enable_sig = 1'b0;
							coeff_enable_sig = 1'b0;
							data_enable_sig = 1'b0;
							Clear_sig = 1'b0;
							work_enable_sig = 1'b0;
							InReady_sig = 1'b1;
							sleep_flag_sig = 1'b0;
							count_rj_reg = 4'd0;
							count_coeff_reg = 9'd0;
							count_data_reg = 8'd0;
							tk_reg = 1'b0;
							if (Frame_sig == 1'b1)
								state_next_reg = Read_rj;
							else
								state_next_reg = Wait_rj;
						end
						
			Read_rj:	begin
							coeffwrite_reg = 9'd0;
							datawrite_reg = 8'd0;
							coeff_enable_sig = 1'b0;
							data_enable_sig = 1'b0;
							Clear_sig = 1'b0;
							work_enable_sig = 1'b0;
							InReady_sig = 1'b1;
							sleep_flag_sig = 1'b0;
							count_coeff_reg = 9'd0;
							count_data_reg = 8'd0;
							if (in_flag_sig == 1'b1 && tk_reg == 1'b0)
							begin
								if (count_rj_reg < 5'd16)
								begin
									rj_enable_sig = 1'b1;
									rjwrite_reg = count_rj_reg;
									count_rj_reg = count_rj_reg + 1'b1;
									state_next_reg = Read_rj;
									tk_reg = 1'b1;
								end
								if (count_rj_reg == 5'd16)
								begin
									state_next_reg = Wait_coeff;
								end
								else
									state_next_reg = Read_rj;
							end
							else if (in_flag_sig == 1'b0)
							begin
								tk_reg = 1'b0;
								rj_enable_sig = 1'b0;
								rjwrite_reg = rjwrite_reg;
								state_next_reg = Read_rj;
							end
							else
								state_next_reg = Read_rj;
						end
			
			Wait_coeff: 
							begin
								rjwrite_reg = 4'd0;
								coeffwrite_reg = 9'd0;
								datawrite_reg = 8'd0;
								rj_enable_sig = 1'b0;
								coeff_enable_sig = 1'b0;
								data_enable_sig = 1'b0;
								Clear_sig = 1'b0;
								work_enable_sig = 1'b0;
								InReady_sig = 1'b1;
								sleep_flag_sig = 1'b0;
								count_coeff_reg = 9'd0;
								count_data_reg = 8'd0;
								if (Frame_sig == 1'b1)
									state_next_reg = Read_coeff;
								else
									state_next_reg = Wait_coeff;
							end
						
			Read_coeff: begin
								rjwrite_reg = 4'd0;
								datawrite_reg = 8'd0;
								rj_enable_sig = 1'b0;
								data_enable_sig = 1'b0;
								Clear_sig = 1'b0;
								work_enable_sig = 1'b0;
								InReady_sig = 1'b1;
								sleep_flag_sig = 1'b0;
								count_data_reg = 8'd0;
								if (in_flag_sig == 1'b1 && tk_reg == 1'b0)
								begin
									if (count_coeff_reg < 10'h200)
									begin
										coeff_enable_sig = 1'b1;
										coeffwrite_reg = count_coeff_reg;
										count_coeff_reg = count_coeff_reg + 1'b1;
										state_next_reg = Read_coeff;
										tk_reg = 1'b1;
									end
									if (count_coeff_reg == 10'h200)
										state_next_reg = Wait_input;
									else
										state_next_reg = Read_coeff;
								end
								else if (in_flag_sig == 1'b0)
								begin
									tk_reg = 1'b0;
									coeff_enable_sig = 1'b0;
									coeffwrite_reg = coeffwrite_reg;
									state_next_reg = Read_coeff;
								end
								else
									state_next_reg = Read_coeff;
							end

			Wait_input: begin
								rjwrite_reg = 4'd0;
								coeffwrite_reg = 9'd0;
								datawrite_reg = 8'd0;
								rj_enable_sig = 1'b0;
								coeff_enable_sig = 1'b0;
								data_enable_sig = 1'b0;
								Clear_sig = 1'b0;
								work_enable_sig = 1'b0;
								InReady_sig = 1'b1;
								sleep_flag_sig = 1'b0;
								count_data_reg = 8'd0;
								if (Reset_n_sig == 1'b0)
									state_next_reg = Reset;
								else if (Frame_sig == 1'b1)
									state_next_reg = Working;
								else
									state_next_reg = Wait_input;
							end
		
			Working:	begin
							rjwrite_reg = 4'd0;
							coeffwrite_reg = 9'd0;
							rj_enable_sig = 1'b0;
							coeff_enable_sig = 1'b0;
							Clear_sig = 1'b0;
							InReady_sig = 1'b1;
							sleep_flag_sig = 1'b0;
							if (Reset_n_sig == 1'b0)
							begin
								Clear_sig = 1'b1;
								state_next_reg = Reset;								
							end
							else if (in_flag_sig == 1'b1 && tk_reg == 1'b0)
							begin
								if (flag_zeroL_sig && flag_zeroR_sig)
								begin
									state_next_reg = Sleep;
									sleep_flag_sig = 1'b1;
								end
								else
								begin
									data_enable_sig = 1'b1;
									datawrite_reg = count_data_reg;
									count_data_reg = count_data_reg + 1'b1;
									count_reg = count_reg + 1'b1;
									state_next_reg = Working;
									work_enable_sig = 1'b1;
									tk_reg = 1'b1;
								end
							end
							else if (in_flag_sig == 1'b0)
							begin
								tk_reg = 1'b0;
								data_enable_sig = 1'b0;
								datawrite_reg = datawrite_reg;
								work_enable_sig = 1'b0;
								state_next_reg = Working;
							end
							else
							begin
								data_enable_sig = 1'b0;
								datawrite_reg = datawrite_reg;
								state_next_reg = Working;
								work_enable_sig = 1'b0;
							end
						end
			
			Reset:	begin
							rjwrite_reg = 4'd0;
							coeffwrite_reg = 9'd0;
							datawrite_reg = 8'd0;
							rj_enable_sig = 1'b0;
							coeff_enable_sig = 1'b0;
							data_enable_sig = 1'b0;
							Clear_sig = 1'b1;
							work_enable_sig = 1'b0;
							InReady_sig = 1'b0;
							sleep_flag_sig = 1'b0;
							count_data_reg = 8'd0;
							tk_reg = 1'b0;
							if (Reset_n_sig == 1'b0)
								state_next_reg = Reset;
							else
								state_next_reg = Wait_input;
						end
			
			Sleep:	begin
							rjwrite_reg = 4'd0;
							coeffwrite_reg = 9'd0;
							datawrite_reg = datawrite_reg;
							rj_enable_sig = 1'b0;
							coeff_enable_sig = 1'b0;
							data_enable_sig = 1'b0;
							Clear_sig = 1'b0;
							work_enable_sig = 1'b0;
							InReady_sig = 1'b1;
							sleep_flag_sig = 1'b1;
							if (Reset_n_sig == 1'b0)
								state_next_reg = Reset;
							else if (in_flag_sig == 1'b1 && tk_reg == 1'b0)
							begin
								if (flag_zeroL_sig && flag_zeroR_sig)
									state_next_reg = Sleep;
								else
								begin
									tk_reg = 1'b1;
									data_enable_sig = 1'b1;
									work_enable_sig = 1'b1;
									sleep_flag_sig = 1'b0;
									datawrite_reg = count_data_reg;
									count_data_reg = count_data_reg + 1'b1;
									count_reg = count_reg + 1'b1;
									state_next_reg = Working;
								end
							end
							else
								state_next_reg = Sleep;
						end
					
		endcase
		end
	end
endmodule


/*
module rj_memory (input write_enable_sig, read_enable_sig, Sclk,
						input [15:0] in_data,
                                           input [3:0] rjwrite, rjread,	
						output [15:0] data_rj_reg);
	reg [15:0] rjmem [0:15];
	always @(posedge Sclk)
	begin
		if(write_enable_sig == 1'b1)
			rjmem[rjwrite] = in_data;
		else
			rjmem[rjwrite] = rjmem[rjwrite];		
	end

	assign data_rj_reg = (read_enable_sig) ? rjmem[rjread] : 16'd0;
endmodule

*/

module shift_accumulator(
    input shift_en_sig,
    input load_sig,
    input clear_sig,
    input Sclk,
    input [39:0] in_bk,
    output [39:0] out_bk_sig
    );

	reg [39:0] shift_reg_reg;
	
	always @(negedge Sclk)
	begin
		if (clear_sig)
			shift_reg_reg = 40'd0;
		if (load_sig && shift_en_sig)
			shift_reg_reg = {in_bk[39], in_bk[39:1]};
		else if (load_sig && !shift_en_sig)
			shift_reg_reg = in_bk;
		else
			shift_reg_reg = shift_reg_reg;
	end

	assign out_bk_sig = shift_reg_reg;
			
endmodule




module top(input Dclk, Sclk, Reset_n_sig, Frame_sig, Start_sig, InputLeft_sig, InputRight_sig,
				 output InReady_sig, OutReady_sig, OutputL_sig, OutputR_sig);
				 
	
	//Wires for memories
	wire rj_enable_sig, coeff_enable_sig, data_enable_sig;				// For main controller
	wire rjL_enable_sig, coeffL_enable_sig, inputL_enable_sig; 		// For ALU controller
	wire rjR_enable_sig, coeffR_enable_sig, inputR_enable_sig;
	wire [3:0] rjwrite_reg, rjL_addr_reg, rjR_addr_reg;
	wire [8:0] coeffwrite_reg, coeffL_addr_reg, coeffR_addr_reg;
	wire [7:0] datawrite_reg, inputL_addr_reg, inputR_addr_reg;
	wire [7:0] rjdataL_sig;
wire [8:0] coeffdataL_sig;
wire [15:0] indataL_sig;
	wire [7:0] rjdataR_sig;
wire [8:0] coeffdataR_sig;
wire [15:0] indataR_sig;
	wire flag_zeroL_sig, flag_zeroR_sig;
	
	//Wires for ALU controller
	wire [39:0] addL_input_sig, addR_input_sig;
	wire addsubL_sig, adderL_en_sig, shiftL_en_sig, loadL_sig, clearL_sig, p2sL_en_sig;
	wire addsubR_sig, adderR_en_sig, shiftR_en_sig, loadR_sig, clearR_sig, p2sR_en_sig;
	
	//Wires for adder, shifter blocks
	wire [39:0] ShiftCompleteL_sig, ShiftCompleteR_sig, sumL_sig, sumR_sig;
	
	//Wires for PISO
	wire OutReadyL_sig, OutReadyR_sig;
	
	//Wires for SIPO
	wire Frame_in_sig, Dclk_in_sig, Clear_sig, in_flag_sig;
	wire [15:0] dataL_reg, dataR_reg;
	
//Wires for main controller
	wire work_enable_sig, sleep_flag_sig, Sclk_in_sig;

//assign addL_input_sig = (indataL_sig[15]) ? {8'hFF, indataL_sig, 16'h0000} : {8'h00, indataL_sig, 16'h0000};
//	assign addR_input_sig = (indataR_sig[15]) ? {8'hFF, indataR_sig, 16'h0000} : {8'h00, indataR_sig, 16'h0000};
	
	
	//////////wires for rj
  wire  [3:0]  rjL_RW0_addr;
  wire         rjL_RW0_clk;
  wire [7:0]  rjL_RW0_wdata;
  wire [7:0]  rjL_RW0_rdata;
  wire         rjL_RW0_en;
  wire         rjL_RW0_wmode;

  wire  [3:0]  rjR_RW0_addr;
  wire         rjR_RW0_clk;
  wire [7:0]  rjR_RW0_wdata;
  wire [7:0]  rjR_RW0_rdata;
  wire         rjR_RW0_en;
  wire         rjR_RW0_wmode;

  assign rjL_RW0_addr = rjwrite_reg;
  assign rjL_RW0_clk = ~Sclk_in_sig;
  assign rjL_RW0_wdata = (rj_enable_sig ? (dataL_reg[7:0]) : (8'd0));
  assign rjL_RW0_rdata = rjdataL_sig[7:0];
  //assign rjL_RW0_rdata = rjdataL_sig[7:0];
  assign rjL_RW0_en = 1'b1;
  assign rjL_RW0_wmode = rj_enable_sig;

  assign rjR_RW0_addr = rjwrite_reg ;
  assign rjR_RW0_clk = ~Sclk_in_sig;
  assign rjR_RW0_wdata = (rj_enable_sig ? (dataR_reg[7:0]) : (8'd0));
  assign rjR_RW0_rdata = rjdataR_sig[7:0];
  assign rjR_RW0_en = 1'b1;
  assign rjR_RW0_wmode = rj_enable_sig;



//////////wires for coeff
  wire  [8:0]  coeffL_RW0_addr;
  wire         coeffL_RW0_clk;
  wire [8:0]  coeffL_RW0_wdata;
  wire [8:0]  coeffL_RW0_rdata;
  wire         coeffL_RW0_en;
  wire         coeffL_RW0_wmode;

  wire  [8:0]  coeffR_RW0_addr;
  wire         coeffR_RW0_clk;
  wire [8:0]  coeffR_RW0_wdata;
  wire [8:0]  coeffR_RW0_rdata;
  wire         coeffR_RW0_en;
  wire         coeffR_RW0_wmode;

  assign coeffL_RW0_addr = coeffwrite_reg;
  assign coeffL_RW0_clk = ~Sclk_in_sig;
  assign coeffL_RW0_wdata = (coeff_enable_sig ? (dataL_reg[8:0]) : (9'd0));
  assign coeffL_RW0_rdata = coeffdataL_sig[8:0];
  assign coeffL_RW0_en = 1'b1; 
  assign coeffL_RW0_wmode = coeff_enable_sig;

  assign coeffR_RW0_addr = coeffwrite_reg;
  assign coeffR_RW0_clk = ~Sclk_in_sig;
  assign coeffR_RW0_wdata = (coeff_enable_sig ? (dataR_reg[8:0]) : (9'd0));
  assign coeffR_RW0_rdata = coeffdataR_sig[8:0];
  assign coeffR_RW0_en = 1'b1;
  assign coeffR_RW0_wmode = coeff_enable_sig;



//////////wires for data
  wire  [7:0]  dataL_RW0_addr;
  wire         dataL_RW0_clk;
  wire [15:0]  dataL_RW0_wdata;
  wire [15:0]  dataL_RW0_rdata;
  wire         dataL_RW0_en;
  wire         dataL_RW0_wmode;

  wire  [7:0]  dataR_RW0_addr;
  wire         dataR_RW0_clk;
  wire [15:0]  dataR_RW0_wdata;
  wire [15:0]  dataR_RW0_rdata;
  wire         dataR_RW0_en;
  wire         dataR_RW0_wmode;

  assign dataL_RW0_addr = datawrite_reg;
  assign dataL_RW0_clk = ~Sclk_in_sig;
  assign dataL_RW0_wdata = (data_enable_sig ? (dataL_reg) : (16'd0));
  assign dataL_RW0_rdata = indataL_sig;
  assign dataL_RW0_en = 1'b1;
  assign dataL_RW0_wmode = data_enable_sig;

  assign dataR_RW0_addr = datawrite_reg;
  assign dataR_RW0_clk = ~Sclk_in_sig;
  assign dataR_RW0_wdata = (data_enable_sig ? (dataR_reg) : (16'd0));
  assign dataR_RW0_rdata = indataR_sig;
  assign dataR_RW0_en = 1'b1;
  assign dataR_RW0_wmode = data_enable_sig;



								 
R_MEM r_mem_L(.RW0_addr(rjL_RW0_addr),.RW0_clk(rjL_RW0_clk),.RW0_wdata(rjL_RW0_wdata),
		.RW0_rdata(rjL_RW0_rdata),.RW0_en(rjL_RW0_en),.RW0_wmode(rjL_RW0_wmode));

R_MEM r_mem_R(.RW0_addr(rjR_RW0_addr),.RW0_clk(rjR_RW0_clk),.RW0_wdata(rjR_RW0_wdata),
		.RW0_rdata(rjR_RW0_rdata),.RW0_en(rjR_RW0_en),.RW0_wmode(rjR_RW0_wmode));


CO_MEM co_mem_L(.RW0_addr(coeffL_RW0_addr),.RW0_clk(coeffL_RW0_clk),.RW0_wdata(coeffL_RW0_wdata),
		.RW0_rdata(coeffL_RW0_rdata),.RW0_en(coeffL_RW0_en),.RW0_wmode(coeffL_RW0_wmode));

CO_MEM co_mem_R(.RW0_addr(coeffR_RW0_addr),.RW0_clk(coeffR_RW0_clk),.RW0_wdata(coeffR_RW0_wdata),
		.RW0_rdata(coeffR_RW0_rdata),.RW0_en(coeffR_RW0_en),.RW0_wmode(coeffR_RW0_wmode));


DATA_MEM data_mem_L(.RW0_addr(dataL_RW0_addr),.RW0_clk(dataL_RW0_clk),.RW0_wdata(dataL_RW0_wdata),
		.RW0_rdata(dataL_RW0_rdata),.RW0_en(dataL_RW0_en),.RW0_wmode(dataL_RW0_wmode));

DATA_MEM data_mem_R(.RW0_addr(dataR_RW0_addr),.RW0_clk(dataR_RW0_clk),.RW0_wdata(dataR_RW0_wdata),
		.RW0_rdata(dataR_RW0_rdata),.RW0_en(dataR_RW0_en),.RW0_wmode(dataR_RW0_wmode));


	//Module instantiations
	
	/*
	rj_memory rjL (.write_enable_sig(rj_enable_sig), .read_enable_sig(rjL_enable_sig), .Sclk(Sclk_in_sig),
					.rjwrite(rjwrite_reg), .rjread(rjL_addr_reg),
					.in_data(dataL_reg), .data_rj_reg(rjdataL_sig));
	
	rj_memory rjR (.write_enable_sig(rj_enable_sig), .read_enable_sig(rjR_enable_sig), .Sclk(Sclk_in_sig),
					.rjwrite(rjwrite_reg), .rjread(rjR_addr_reg),
					.in_data(dataR_reg), .data_rj_reg(rjdataR_sig));
					
	coeff_memory coeffL (.write_enable_sig(coeff_enable_sig), .read_enable_sig(coeffL_enable_sig), .Sclk(Sclk_in_sig),
						  .coeffwrite(coeffwrite_reg), .coeffread(coeffL_addr_reg),
						  .in_data(dataL_reg), .data_coeff_reg(coeffdataL_sig));
	
	coeff_memory coeffR (.write_enable_sig(coeff_enable_sig), .read_enable_sig(coeffR_enable_sig), .Sclk(Sclk_in_sig),
						  .coeffwrite(coeffwrite_reg), .coeffread(coeffR_addr_reg),
						  .in_data(dataR_reg), .data_coeff_reg(coeffdataR_sig));

	data_memory inL (.write_enable_sig(data_enable_sig), .read_enable_sig(inputL_enable_sig), .Sclk(Sclk_in_sig), .in_flag_sig(in_flag_sig),
					   .datawrite(datawrite_reg), .dataread(inputL_addr_reg),
					   .in_data(dataL_reg), .input_data_reg(indataL_sig), .flag_zero_reg(flag_zeroL_sig));

	data_memory inR (.write_enable_sig(data_enable_sig), .read_enable_sig(inputR_enable_sig), .Sclk(Sclk_in_sig), .in_flag_sig(in_flag_sig),
					   .datawrite(datawrite_reg), .dataread(inputR_addr_reg),
					   .in_data(dataR_reg), .input_data_reg(indataR_sig), .flag_zero_reg(flag_zeroR_sig));
	*/		   
	main_controller main_ctrl (.Sclk(Sclk), .Dclk(Dclk), .Start_sig(Start_sig), .Reset_n_sig(Reset_n_sig),
								.Frame_sig(Frame_sig), .in_flag_sig(in_flag_sig), .flag_zeroL_sig(flag_zeroL_sig), .flag_zeroR_sig(flag_zeroR_sig),
								.rjwrite_reg(rjwrite_reg), .coeffwrite_reg(coeffwrite_reg), .datawrite_reg(datawrite_reg),
								.rj_enable_sig(rj_enable_sig), .coeff_enable_sig(coeff_enable_sig), .data_enable_sig(data_enable_sig), .Clear_sig(Clear_sig),
								.Frame_out_sig(Frame_in_sig), .Dclk_out_sig(Dclk_in_sig), .Sclk_out_sig(Sclk_in_sig),
								.work_enable_sig(work_enable_sig), .sleep_flag_sig(sleep_flag_sig),
								.InReady_sig(InReady_sig));
	
	alu_controller alu_ctrl (.work_enable_sig(work_enable_sig), .Clear_sig(Clear_sig), .Sclk(Sclk_in_sig), .sleep_flag_sig(sleep_flag_sig),
							 .rjdataL_sig(rjL_RW0_rdata), .coeffdataL_sig(coeffL_RW0_rdata), .indataL_sig(dataL_RW0_rdata),
							 .rjdataR_sig(rjR_RW0_rdata), .coeffdataR_sig(coeffR_RW0_rdata), .indataR_sig(dataR_RW0_rdata),
							 .addL_input_sig(addL_input_sig), .addR_input_sig(addR_input_sig),
							 .rjL_addr_reg(rjL_addr_reg), .coeffL_addr_reg(coeffL_addr_reg), .inputL_addr_reg(inputL_addr_reg),
							 .rjR_addr_reg(rjR_addr_reg), .coeffR_addr_reg(coeffR_addr_reg), .inputR_addr_reg(inputR_addr_reg),
							 .rjL_enable_sig(rjL_enable_sig), .coeffL_enable_sig(coeffL_enable_sig), .inputL_enable_sig(inputL_enable_sig),
							 .rjR_enable_sig(rjR_enable_sig), .coeffR_enable_sig(coeffR_enable_sig), .inputR_enable_sig(inputR_enable_sig),
						.addsubL_sig(addsubL_sig), .adderL_en_sig(adderL_en_sig), .shiftL_en_sig(shiftL_en_sig), .loadL_sig(loadL_sig), .clearL_sig(clearL_sig), .p2sL_en_sig(p2sL_en_sig),
						 .addsubR_sig(addsubR_sig), .adderR_en_sig(adderR_en_sig), .shiftR_en_sig(shiftR_en_sig), .loadR_sig(loadR_sig), .clearR_sig(clearR_sig), .p2sR_en_sig(p2sR_en_sig));
							 
	adder addL (.a_sig(addL_input_sig), .b_sig(ShiftCompleteL_sig), .addsub_sig(addsubL_sig), .adder_en_sig(adderL_en_sig), .sum_sig(sumL_sig));
	
	adder addR (.a_sig(addR_input_sig), .b_sig(ShiftCompleteR_sig), .addsub_sig(addsubR_sig), .adder_en_sig(adderR_en_sig), .sum_sig(sumR_sig));
	
	shift_accumulator shiftL (.shift_en_sig(shiftL_en_sig), .load_sig(loadL_sig), .clear_sig(clearL_sig), .Sclk(Sclk_in_sig), .in_bk(sumL_sig), .out_bk_sig(ShiftCompleteL_sig));
	
	shift_accumulator shiftR (.shift_en_sig(shiftR_en_sig), .load_sig(loadR_sig), .clear_sig(clearR_sig), .Sclk(Sclk_in_sig), .in_bk(sumR_sig), .out_bk_sig(ShiftCompleteR_sig));
	
	PISO PISOL (.Sclk(Sclk_in_sig), .Clear_sig(Clear_sig), .Frame_sig(Frame_in_sig), .ShiftComplete_sig(ShiftCompleteL_sig), .SerialOut_sig(OutputL_sig), .p2s_en_sig(p2sL_en_sig), .OutReady_sig(OutReadyL_sig));
	
	PISO PISOR (.Sclk(Sclk_in_sig), .Clear_sig(Clear_sig), .Frame_sig(Frame_in_sig), .ShiftComplete_sig(ShiftCompleteR_sig), .SerialOut_sig(OutputR_sig), .p2s_en_sig(p2sR_en_sig), .OutReady_sig(OutReadyR_sig));

	SIPO Sipo (.Frame_sig(Frame_in_sig), .Dclk(Dclk_in_sig), .Clear_sig(Clear_sig),
					.InputLeft_sig(InputLeft_sig), .InputRight_sig(InputRight_sig), .in_flag_sig(in_flag_sig),
					.dataL_reg(dataL_reg), .dataR_reg(dataR_reg));
	assign OutReady_sig = OutReadyL_sig || OutReadyR_sig;
	
endmodule



module R_MEM(
  input  [3:0]  RW0_addr,
  input         RW0_clk,
  input  [7:0]  RW0_wdata,
  output [7:0]  RW0_rdata,
  input         RW0_en,
  input         RW0_wmode
);
  wire [3:0] mem_0_0_A1;
  wire  mem_0_0_CE1;
  wire [7:0] mem_0_0_I1;
  wire [7:0] mem_0_0_O1;
  wire  mem_0_0_CSB1;
  wire  mem_0_0_OEB1;
  wire  mem_0_0_WEB1;
  wire [3:0] mem_0_0_A2;
  wire  mem_0_0_CE2;
  wire [7:0] mem_0_0_I2;
  wire [7:0] mem_0_0_O2;
  wire  mem_0_0_CSB2;
  wire  mem_0_0_OEB2;
  wire  mem_0_0_WEB2;
	//assign mem_0_0_O2 = 8'd0;
  SRAM2RW16x8 mem_0_0 (
    .A1(mem_0_0_A1),
    .CE1(mem_0_0_CE1),
    .I1(mem_0_0_I1),
    .O1(mem_0_0_O1),
    .CSB1(mem_0_0_CSB1),
    .OEB1(mem_0_0_OEB1),
    .WEB1(mem_0_0_WEB1),
    .A2(mem_0_0_A2),
    .CE2(mem_0_0_CE2),
    .I2(mem_0_0_I2),
    .O2(mem_0_0_O2),
    .CSB2(mem_0_0_CSB2),
    .OEB2(mem_0_0_OEB2),
    .WEB2(mem_0_0_WEB2)
  );
  assign RW0_rdata = mem_0_0_O1;
  assign mem_0_0_A1 = RW0_addr;
  assign mem_0_0_CE1 = RW0_clk;
  assign mem_0_0_I1 = RW0_wdata[7:0];
  assign mem_0_0_CSB1 = ~RW0_en;
  assign mem_0_0_OEB1 = ~(~RW0_wmode & RW0_en);
  assign mem_0_0_WEB1 = ~RW0_wmode;
  assign mem_0_0_A2 = RW0_addr;
  assign mem_0_0_CE2 = 'b1;
  assign mem_0_0_I2 = RW0_wdata[7:0];
  assign mem_0_0_CSB2 = 'b1;
  assign mem_0_0_OEB2 = 'b1;
  assign mem_0_0_WEB2 = 'b1;
/*
  reg [15:0] rjmem [0:15];

    always @(posedge RW0_clk) begin
        if (RW0_en && RW0_wmode) begin
            rjmem[RW0_addr] <= RW0_wdata;
        end
    end

    assign RW0_rdata = RW0_en && ~RW0_wmode ? rjmem[RW0_addr] : 8'd0;
 
*/

endmodule

module CO_MEM(
  input  [8:0]  RW0_addr,
  input         RW0_clk,
  input  [8:0] RW0_wdata,
  output [8:0] RW0_rdata,
  input         RW0_en,
  input         RW0_wmode
);
  wire [11:0] mem_0_O,mem_1_O,mem_2_O,mem_3_O;
  wire mem_OEB;
  wire mem_WEB;
  wire mem_0_CSB, mem_1_CSB, mem_2_CSB, mem_3_CSB;
  
  SRAM1RW128x12 mem_0_0 (
    .A(RW0_addr[6:0]),
    .CE(RW0_clk),
    .I( {3'b000, RW0_wdata} ),
    .O(mem_0_O),
    .CSB(mem_0_CSB),
    .OEB(mem_OEB),
    .WEB(mem_WEB)
  );
  SRAM1RW128x12 mem_0_1 (
    .A(RW0_addr[6:0]),
    .CE(RW0_clk),
    .I( {3'b000, RW0_wdata} ),
    .O(mem_1_O),
    .CSB(mem_1_CSB),
    .OEB(mem_OEB),
    .WEB(mem_WEB)
  );
  SRAM1RW128x12 mem_0_2 (
    .A(RW0_addr[6:0]),
    .CE(RW0_clk),
    .I( {3'b000, RW0_wdata} ),
    .O(mem_2_O),
    .CSB(mem_2_CSB),
    .OEB(mem_OEB),
    .WEB(mem_WEB)
  );
  SRAM1RW128x12 mem_0_3 (
    .A(RW0_addr[6:0]),
    .CE(RW0_clk),
    .I( {3'b000, RW0_wdata} ),
    .O(mem_3_O),
    .CSB(mem_3_CSB),
    .OEB(mem_OEB),
    .WEB(mem_WEB)
  );
  assign RW0_rdata = {9{~mem_OEB}} & (({9{~mem_0_CSB}} & mem_0_O[8:0]) | ({9{~mem_1_CSB}} & mem_1_O[8:0]) | ({9{~mem_2_CSB}} & mem_2_O[8:0]) | ({9{~mem_3_CSB}} & mem_3_O[8:0]));
  assign mem_OEB = ~(RW0_en & ~RW0_wmode);
  assign mem_WEB = ~(RW0_en & RW0_wmode);
  assign mem_0_CSB = ~(RW0_addr[8:7] == 2'd0);
  assign mem_1_CSB = ~(RW0_addr[8:7] == 2'd1);
  assign mem_2_CSB = ~(RW0_addr[8:7] == 2'd2);
  assign mem_3_CSB = ~(RW0_addr[8:7] == 2'd3);
/*
reg [15:0] coeffmem [0:511];

    always @(posedge RW0_clk) begin
        if (RW0_en && RW0_wmode) begin
            coeffmem[RW0_addr] <= RW0_wdata;
        end
    end

    assign data_coeff_reg = (RW0_en && ~RW0_wmode) ? coeffmem[RW0_addr] : 16'd0;
*/


 endmodule


module DATA_MEM(
  input  [7:0]  RW0_addr,
  input         RW0_clk,
  input  [15:0] RW0_wdata,
  output [15:0] RW0_rdata,
  input         RW0_en,
  input         RW0_wmode
);
  wire [7:0] mem_0_0_A;
  wire  mem_0_0_CE;
  wire [7:0] mem_0_0_I;
  wire [7:0] mem_0_0_O;
  wire  mem_0_0_CSB;
  wire  mem_0_0_OEB;
  wire  mem_0_0_WEB;
  wire [7:0] mem_0_1_A;
  wire  mem_0_1_CE;
  wire [7:0] mem_0_1_I;
  wire [7:0] mem_0_1_O;
  wire  mem_0_1_CSB;
  wire  mem_0_1_OEB;
  wire  mem_0_1_WEB;
  SRAM1RW256x8 mem_0_0 (
    .A(mem_0_0_A),
    .CE(mem_0_0_CE),
    .I(mem_0_0_I),
    .O(mem_0_0_O),
    .CSB(mem_0_0_CSB),
    .OEB(mem_0_0_OEB),
    .WEB(mem_0_0_WEB)
  );
  SRAM1RW256x8 mem_0_1 (
    .A(mem_0_1_A),
    .CE(mem_0_1_CE),
    .I(mem_0_1_I),
    .O(mem_0_1_O),
    .CSB(mem_0_1_CSB),
    .OEB(mem_0_1_OEB),
    .WEB(mem_0_1_WEB)
  );
  assign RW0_rdata = {mem_0_1_O,mem_0_0_O};
  assign mem_0_0_A = RW0_addr;
  assign mem_0_0_CE = RW0_clk;
  assign mem_0_0_I = RW0_wdata[7:0];
  assign mem_0_0_CSB = ~RW0_en;
  assign mem_0_0_OEB = ~(~RW0_wmode & RW0_en);
  assign mem_0_0_WEB = ~RW0_wmode;
  assign mem_0_1_A = RW0_addr;
  assign mem_0_1_CE = RW0_clk;
  assign mem_0_1_I = RW0_wdata[15:8];
  assign mem_0_1_CSB = ~RW0_en;
  assign mem_0_1_OEB = ~(~RW0_wmode & RW0_en);
  assign mem_0_1_WEB = ~RW0_wmode;
/*
 reg [15:0] datamem [0:255];
    reg [11:0] count_zero_reg;
    reg [11:0] ani_cnt_reg;
    reg flag_zero_reg;

    always @(posedge RW0_clk) begin
        if (RW0_en && RW0_wmode) begin
            ani_cnt_reg <= ani_cnt_reg + 1'b1;
            datamem[RW0_addr] <= RW0_wdata;
        end

        if (RW0_wdata == 16'd0 && RW0_en && RW0_wmode) begin
            count_zero_reg <= count_zero_reg + 1'b1;
            if (count_zero_reg == 10'd800 || count_zero_reg > 10'd800) begin
                flag_zero_reg <= 1'b1;
            end else if (count_zero_reg > 10'd800) begin
                count_zero_reg <= 10'd800;
                flag_zero_reg <= 1'b1;
            end
        end else begin
            count_zero_reg <= 12'd0;
            flag_zero_reg <= 1'b0;
        end
    end

    assign RW0_rdata = RW0_en && ~RW0_wmode ? datamem[RW0_addr] : 16'd0;

*/
 endmodule

module SRAM1RW128x12 (A,CE,WEB,OEB,CSB,I,O);

input CE;
input WEB;
input OEB;
input CSB;

input  [7-1:0] A;
input  [12-1:0] I;
output [12-1:0] O;

reg     [12-1:0] memory[128-1:0];
reg     [12-1:0] data_out;
wire    [12-1:0] O;

wire RE;
wire WE;
and u1 (RE, ~CSB, ~OEB);
and u2 (WE, ~CSB, ~WEB);

always @ (posedge CE) begin
    if (RE)
        data_out <= memory[A];
    if (WE)
        memory[A] <= I;
end

//Delete the specify block if reported error 
reg NOTIFIER;
specify
$setuphold(posedge CE, posedge I[0], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[0], 0, 0, NOTIFIER);
(CE => O[0]) = 0;
$setuphold(posedge CE, posedge I[1], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[1], 0, 0, NOTIFIER);
(CE => O[1]) = 0;
$setuphold(posedge CE, posedge I[2], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[2], 0, 0, NOTIFIER);
(CE => O[2]) = 0;
$setuphold(posedge CE, posedge I[3], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[3], 0, 0, NOTIFIER);
(CE => O[3]) = 0;
$setuphold(posedge CE, posedge I[4], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[4], 0, 0, NOTIFIER);
(CE => O[4]) = 0;
$setuphold(posedge CE, posedge I[5], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[5], 0, 0, NOTIFIER);
(CE => O[5]) = 0;
$setuphold(posedge CE, posedge I[6], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[6], 0, 0, NOTIFIER);
(CE => O[6]) = 0;
$setuphold(posedge CE, posedge I[7], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[7], 0, 0, NOTIFIER);
(CE => O[7]) = 0;
$setuphold(posedge CE, posedge I[8], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[8], 0, 0, NOTIFIER);
(CE => O[8]) = 0;
$setuphold(posedge CE, posedge I[9], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[9], 0, 0, NOTIFIER);
(CE => O[9]) = 0;
$setuphold(posedge CE, posedge I[10], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[10], 0, 0, NOTIFIER);
(CE => O[10]) = 0;
$setuphold(posedge CE, posedge I[11], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[11], 0, 0, NOTIFIER);
(CE => O[11]) = 0;
$setuphold(posedge CE, posedge A[0], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[0], 0, 0, NOTIFIER);
$setuphold(posedge CE, posedge A[1], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[1], 0, 0, NOTIFIER);
$setuphold(posedge CE, posedge A[2], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[2], 0, 0, NOTIFIER);
$setuphold(posedge CE, posedge A[3], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[3], 0, 0, NOTIFIER);
$setuphold(posedge CE, posedge A[4], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[4], 0, 0, NOTIFIER);
$setuphold(posedge CE, posedge A[5], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[5], 0, 0, NOTIFIER);
$setuphold(posedge CE, posedge A[6], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[6], 0, 0, NOTIFIER);

endspecify

assign O = data_out;

endmodule

module SRAM1RW256x8 (A,CE,WEB,OEB,CSB,I,O);

input CE;
input WEB;
input OEB;
input CSB;

input  [8-1:0] A;
input  [8-1:0] I;
output [8-1:0] O;

reg     [8-1:0] memory[256-1:0];
reg     [8-1:0] data_out;
wire    [8-1:0] O;

wire RE;
wire WE;
and u1 (RE, ~CSB, ~OEB);
and u2 (WE, ~CSB, ~WEB);

always @ (posedge CE) begin
    if (RE)
        data_out <= memory[A];
    if (WE)
        memory[A] <= I;
end

//Delete the specify block if reported error
reg NOTIFIER;
specify
$setuphold(posedge CE, posedge I[0], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[0], 0, 0, NOTIFIER);
(CE => O[0]) = 0;
$setuphold(posedge CE, posedge I[1], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[1], 0, 0, NOTIFIER);
(CE => O[1]) = 0;
$setuphold(posedge CE, posedge I[2], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[2], 0, 0, NOTIFIER);
(CE => O[2]) = 0;
$setuphold(posedge CE, posedge I[3], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[3], 0, 0, NOTIFIER);
(CE => O[3]) = 0;
$setuphold(posedge CE, posedge I[4], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[4], 0, 0, NOTIFIER);
(CE => O[4]) = 0;
$setuphold(posedge CE, posedge I[5], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[5], 0, 0, NOTIFIER);
(CE => O[5]) = 0;
$setuphold(posedge CE, posedge I[6], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[6], 0, 0, NOTIFIER);
(CE => O[6]) = 0;
$setuphold(posedge CE, posedge I[7], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge I[7], 0, 0, NOTIFIER);
(CE => O[7]) = 0;
$setuphold(posedge CE, posedge A[0], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[0], 0, 0, NOTIFIER);
$setuphold(posedge CE, posedge A[1], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[1], 0, 0, NOTIFIER);
$setuphold(posedge CE, posedge A[2], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[2], 0, 0, NOTIFIER);
$setuphold(posedge CE, posedge A[3], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[3], 0, 0, NOTIFIER);
$setuphold(posedge CE, posedge A[4], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[4], 0, 0, NOTIFIER);
$setuphold(posedge CE, posedge A[5], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[5], 0, 0, NOTIFIER);
$setuphold(posedge CE, posedge A[6], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[6], 0, 0, NOTIFIER);
$setuphold(posedge CE, posedge A[7], 0, 0, NOTIFIER);
$setuphold(posedge CE, negedge A[7], 0, 0, NOTIFIER);

endspecify

assign O = data_out;

endmodule


module SRAM2RW16x8 (A1,A2,CE1,CE2,WEB1,WEB2,OEB1,OEB2,CSB1,CSB2,I1,I2,O1,O2);

input CE1;
input CE2;
input WEB1;
input WEB2;
input OEB1;
input OEB2;
input CSB1;
input CSB2;

input  [4-1:0]    A1;
input  [4-1:0]    A2;
input  [8-1:0] I1;
input  [8-1:0] I2;
output [8-1:0] O1;
output [8-1:0] O2;

reg     [8-1:0] memory[16-1:0];
reg     [8-1:0] data_out1;
reg     [8-1:0] data_out2;
wire    [8-1:0] O1;
wire    [8-1:0] O2;

wire RE1;
wire RE2;
wire WE1;
wire WE2;
and u1 (RE1, ~CSB1, ~OEB1);
and u2 (RE2, ~CSB2, ~OEB2);
and u3 (WE1, ~CSB1, ~WEB1);
and u4 (WE2, ~CSB2, ~WEB2);

always @ (posedge CE1) begin
    if (RE1)
        data_out1 <= memory[A1];
    if (WE1)
        memory[A1] <= I1;
end

always @ (posedge CE2) begin
    if (RE2)
        data_out2 <= memory[A2];
    if (WE2)
        memory[A2] <= I2;
end

//Delete the specify block if reported error
reg NOTIFIER;
specify
$setuphold(posedge CE1, posedge I1[0], 0, 0, NOTIFIER);
$setuphold(posedge CE2, posedge I2[0], 0, 0, NOTIFIER);
$setuphold(posedge CE1, negedge I1[0], 0, 0, NOTIFIER);
$setuphold(posedge CE2, negedge I2[0], 0, 0, NOTIFIER);
(CE1 => O1[0]) = 0;
(CE2 => O2[0]) = 0;
$setuphold(posedge CE1, posedge I1[1], 0, 0, NOTIFIER);
$setuphold(posedge CE2, posedge I2[1], 0, 0, NOTIFIER);
$setuphold(posedge CE1, negedge I1[1], 0, 0, NOTIFIER);
$setuphold(posedge CE2, negedge I2[1], 0, 0, NOTIFIER);
(CE1 => O1[1]) = 0;
(CE2 => O2[1]) = 0;
$setuphold(posedge CE1, posedge I1[2], 0, 0, NOTIFIER);
$setuphold(posedge CE2, posedge I2[2], 0, 0, NOTIFIER);
$setuphold(posedge CE1, negedge I1[2], 0, 0, NOTIFIER);
$setuphold(posedge CE2, negedge I2[2], 0, 0, NOTIFIER);
(CE1 => O1[2]) = 0;
(CE2 => O2[2]) = 0;
$setuphold(posedge CE1, posedge I1[3], 0, 0, NOTIFIER);
$setuphold(posedge CE2, posedge I2[3], 0, 0, NOTIFIER);
$setuphold(posedge CE1, negedge I1[3], 0, 0, NOTIFIER);
$setuphold(posedge CE2, negedge I2[3], 0, 0, NOTIFIER);
(CE1 => O1[3]) = 0;
(CE2 => O2[3]) = 0;
$setuphold(posedge CE1, posedge I1[4], 0, 0, NOTIFIER);
$setuphold(posedge CE2, posedge I2[4], 0, 0, NOTIFIER);
$setuphold(posedge CE1, negedge I1[4], 0, 0, NOTIFIER);
$setuphold(posedge CE2, negedge I2[4], 0, 0, NOTIFIER);
(CE1 => O1[4]) = 0;
(CE2 => O2[4]) = 0;
$setuphold(posedge CE1, posedge I1[5], 0, 0, NOTIFIER);
$setuphold(posedge CE2, posedge I2[5], 0, 0, NOTIFIER);
$setuphold(posedge CE1, negedge I1[5], 0, 0, NOTIFIER);
$setuphold(posedge CE2, negedge I2[5], 0, 0, NOTIFIER);
(CE1 => O1[5]) = 0;
(CE2 => O2[5]) = 0;
$setuphold(posedge CE1, posedge I1[6], 0, 0, NOTIFIER);
$setuphold(posedge CE2, posedge I2[6], 0, 0, NOTIFIER);
$setuphold(posedge CE1, negedge I1[6], 0, 0, NOTIFIER);
$setuphold(posedge CE2, negedge I2[6], 0, 0, NOTIFIER);
(CE1 => O1[6]) = 0;
(CE2 => O2[6]) = 0;
$setuphold(posedge CE1, posedge I1[7], 0, 0, NOTIFIER);
$setuphold(posedge CE2, posedge I2[7], 0, 0, NOTIFIER);
$setuphold(posedge CE1, negedge I1[7], 0, 0, NOTIFIER);
$setuphold(posedge CE2, negedge I2[7], 0, 0, NOTIFIER);
(CE1 => O1[7]) = 0;
(CE2 => O2[7]) = 0;
$setuphold(posedge CE1, posedge A1[0], 0, 0, NOTIFIER);
$setuphold(posedge CE2, posedge A2[0], 0, 0, NOTIFIER);
$setuphold(posedge CE1, negedge A1[0], 0, 0, NOTIFIER);
$setuphold(posedge CE2, negedge A2[0], 0, 0, NOTIFIER);
$setuphold(posedge CE1, posedge A1[1], 0, 0, NOTIFIER);
$setuphold(posedge CE2, posedge A2[1], 0, 0, NOTIFIER);
$setuphold(posedge CE1, negedge A1[1], 0, 0, NOTIFIER);
$setuphold(posedge CE2, negedge A2[1], 0, 0, NOTIFIER);
$setuphold(posedge CE1, posedge A1[2], 0, 0, NOTIFIER);
$setuphold(posedge CE2, posedge A2[2], 0, 0, NOTIFIER);
$setuphold(posedge CE1, negedge A1[2], 0, 0, NOTIFIER);
$setuphold(posedge CE2, negedge A2[2], 0, 0, NOTIFIER);
$setuphold(posedge CE1, posedge A1[3], 0, 0, NOTIFIER);
$setuphold(posedge CE2, posedge A2[3], 0, 0, NOTIFIER);
$setuphold(posedge CE1, negedge A1[3], 0, 0, NOTIFIER);
$setuphold(posedge CE2, negedge A2[3], 0, 0, NOTIFIER);

endspecify

assign O1 = data_out1;
assign O2 = data_out2;

endmodule 








