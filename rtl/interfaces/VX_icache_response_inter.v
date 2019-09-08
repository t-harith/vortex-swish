
`include "VX_define.v"

`ifndef VX_ICACHE_RSP

`define VX_ICACHE_RSP

interface VX_icache_response_inter ();

	// wire ready;
	// wire stall;
	wire[31:0] instruction;

	// source-side view
	modport snk (
		input instruction
	);


	// source-side view
	modport src (
		output instruction
	);


endinterface


`endif