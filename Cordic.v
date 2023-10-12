//            parameterized CORDIC module 
//
// Description: 
// CORDIC module that works in rotation or vectoring mode depending
// on parameter MODE
//
// ------------- Rotation Mode (MODE = 0)---------------
// used to rotate a vector by a certain angle
// ------------- Vectoring Mode (MODE = 1)---------------
// convert from rectangular to polar coordinates
//
// ------------------ PARAMETERS ------------------
//
// MODE: if 1: module works in vectoring mode
// 		 if 0: module works in rotational mode
// STAGES: number of cordic stages (max 16).
// WIDTH: width of input real and img part.
// PHASE_WIDTH: width of output phase (max 19), first 3 bits represent 
//              decimal part and the rest represent fraction part 
// first 4 bits of output mag shall be storing decimal part

module Cordic #(parameter MODE = 0, STAGES = 8, WIDTH = 16, PHASE_WIDTH = 19)(
input wire                        in_clk,
input wire                        in_rst,
input wire                        in_enable,
input wire    [WIDTH-1:0]         in_x,
input wire    [WIDTH-1:0]         in_y,
input wire    [PHASE_WIDTH-1:0]   in_phase,

output wire                       out_valid,
output wire   [PHASE_WIDTH-1:0]   out_phase,
output wire   [WIDTH-1:0]         out_x,
output wire   [WIDTH-1:0]         out_y
);

localparam	ROTATIONAL = 0, 
			VECTORING  = 1;
			
// each packed array represents a Cordic stage
wire signed   [WIDTH:0]         xn [STAGES:0];
wire signed   [WIDTH:0]         yn [STAGES:0];
wire signed   [PHASE_WIDTH-1:0]   zn [STAGES:0];

// create a register for each stage to apply pipelining
reg signed  [WIDTH:0]         xn_reg [STAGES:0];
reg signed  [WIDTH:0]         yn_reg [STAGES:0];
reg signed  [PHASE_WIDTH-1:0]   zn_reg [STAGES:0];

// delay starting of cordic by 1 cycle
reg start;


generate
	if(MODE == ROTATIONAL) begin
// quad adjustment stage
reg [WIDTH-1:0] temp_x, temp_y;
reg [PHASE_WIDTH-1:0] temp_z;


always @ (posedge in_clk or negedge in_rst) 
begin
 if(!in_rst) begin
	temp_x <= 'b0;
	temp_y <= 'b0;
	temp_z <= 'b0; 
 end else if (start) begin
	temp_x <= in_x;
	temp_y <= in_y;
	temp_z <= in_phase;
	end
end

wire [PHASE_WIDTH-1:0] half_pi = 19'h19220;
wire [PHASE_WIDTH-1:0] z_limit = 19'h1b333;
reg [WIDTH-1:0] adjusted_x, adjusted_y;
reg [PHASE_WIDTH-1:0] adjusted_z;

always @ (*) 
begin
	if(temp_z[PHASE_WIDTH-1] == 1'b0 && temp_z > z_limit) begin
		adjusted_x = ~temp_y + 'b1;
		adjusted_y = temp_x;
		adjusted_z = temp_z - half_pi;
	end 
	else if(temp_z[PHASE_WIDTH-1] == 1'b1 && (~temp_z+19'b1) > z_limit) begin
		adjusted_x = temp_y;
		adjusted_y = ~temp_x + 'b1;
		adjusted_z = temp_z + half_pi;
	end 
	else begin
		adjusted_x = temp_x;
		adjusted_y = temp_y;
		adjusted_z = temp_z;
	end
end
// apply inputs to first stage directly
assign xn[0] = {adjusted_x[WIDTH-1], adjusted_x};
assign yn[0] = {adjusted_y[WIDTH-1], adjusted_y};
assign zn[0] = adjusted_z;

end else if(MODE == VECTORING) begin
// apply inputs to first stage directly
assign xn[0] = {in_x[WIDTH-1], in_x};
assign yn[0] = {in_y[WIDTH-1], in_y};
assign zn[0] = 'b0;
end

endgenerate


// genrate number of cordic stages equal to (STAGES) parameter
generate 
genvar i;
	if(MODE == ROTATIONAL) begin
for(i=0; i<STAGES; i=i+1) begin
  assign xn[i+1] = (zn_reg[i][PHASE_WIDTH-1] == 1'b0) ? xn_reg[i]-(yn_reg[i]>>>i)   : xn_reg[i]+(yn_reg[i]>>>i);
  assign yn[i+1] = (zn_reg[i][PHASE_WIDTH-1] == 1'b0) ? yn_reg[i]+(xn_reg[i]>>>i)   : yn_reg[i]-(xn_reg[i]>>>i);
  assign zn[i+1] = (zn_reg[i][PHASE_WIDTH-1] == 1'b0) ? zn_reg[i]-arctangent(i) : zn_reg[i]+arctangent(i);
end
end else if(MODE == VECTORING) begin
for(i=0; i<STAGES; i=i+1) begin
  assign xn[i+1] = (yn_reg[i][WIDTH] == 1'b1) ? xn_reg[i]-(yn_reg[i]>>>i)   : xn_reg[i]+(yn_reg[i]>>>i);
  assign yn[i+1] = (yn_reg[i][WIDTH] == 1'b1) ? yn_reg[i]+(xn_reg[i]>>>i)   : yn_reg[i]-(xn_reg[i]>>>i);
  assign zn[i+1] = (yn_reg[i][WIDTH] == 1'b1) ? zn_reg[i]-arctangent(i) : zn_reg[i]+arctangent(i);
end
end
endgenerate
// PIPELINING
//
// reg start;
always @ (posedge in_clk or negedge in_rst)
	if(!in_rst)
		start <= 1'b0;
	else
		start <= in_enable;
	
reg [STAGES+2:0] valid_reg;	
// register the value of each stage on positive clock edge
for(i=0; i<=STAGES; i=i+1) begin
  always @ (posedge in_clk or negedge in_rst)
    if(!in_rst)begin
      xn_reg[i] <= 'b0;
      yn_reg[i] <= 'b0;
      zn_reg[i] <= 'b0;    
    end
    else if(valid_reg[i+1]) begin
      xn_reg[i] <= xn[i];
      yn_reg[i] <= yn[i];
      zn_reg[i] <= zn[i];
    end
end

// temp. fixed multiplication 
wire signed [15:0] k = 18'h009B7;
wire signed [31:0] scaled_x = xn_reg[STAGES] * k;
wire signed [31:0] scaled_y = yn_reg[STAGES] * k;


// temporary fixed truncation
assign out_x = scaled_x[27:12];
assign out_y = scaled_y[27:12];
assign out_phase = zn_reg[STAGES];

// when in_enable is high, cordic starts in the next cycle
// valid signal logic
always@(posedge in_clk or negedge in_rst)
begin
  if(!in_rst)
    valid_reg <= 'b0;
  else
    valid_reg <= {valid_reg[STAGES+1:0], in_enable};
end
assign out_valid = valid_reg[STAGES+2];


/*
   arctan table in (3,16) format 19 bits, phases are stored in radian
*/
function [18:0] arctangent;
  input [3:0] i;
  begin
    case (i)
    0 : arctangent = 19'b000_1100100100001111; // phase = 0.78539
    1 : arctangent = 19'b000_0111011010110001; // phase = 0.46364
    2 : arctangent = 19'b000_0011111010110110; // phase = 0.24497
    3 : arctangent = 19'b000_0001111111010101; // phase = 0.12435
    4 : arctangent = 19'b000_0000111111111010; // phase = 0.06241
    5 : arctangent = 19'b000_0000011111111111; // phase = 0.031239
    6 : arctangent = 19'b000_0000001111111111; // phase = 0.01562
    7 : arctangent = 19'b000_0000000111111111; // phase = 0.0078
    8 : arctangent = 19'b000_0000000011111111; // phase = 0.0039
    9 : arctangent = 19'b000_0000000001111111; // phase = 0.00195
    10: arctangent = 19'b000_0000000000111111; // phase = 0.00097
    11: arctangent = 19'b000_0000000000011111; // phase = 0.00048
    12: arctangent = 19'b000_0000000000001111; // phase = 0.00024
    13: arctangent = 19'b000_0000000000000111; // phase = 0.00012
    14: arctangent = 19'b000_0000000000000011; // phase = 0.00006
    15: arctangent = 19'b000_0000000000000001; // phase = 0.00003
    default: arctangent = 19'b000_0000000000000000; // phase = 0
    endcase
  end
endfunction

endmodule



