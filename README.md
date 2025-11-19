# SAFETY SYSTEM USING VERILOG HDL


## Abstract:
This project focuses on developing an automated smoke and temperature monitoring
system using Verilog Hardware Description Language (HDL), intended for deployment in
commercial and industrial settings.The system simulates temperature and smoke sensors
to continuously track environmental conditions and detect when they exceed safe operating
limits. When either the smoke level or temperature surpasses the predefined threshold
values, a finite state machine (FSM) automatically triggers alarm signals and activates the
sprinkler system to address the potential hazard. 

## Introduction:
This project presents a smoke and temperature monitoring system designed using Verilog
HDL for application in industrial and commercial settings. The system uses simulated
temperature and smoke sensors along with comparison logic to check their readings against
predefined safety thresholds.A finite state machine coordinates the automatic activation of
alarms and sprinkler systems when abnormal or hazardous levels are detected, ensuring a
quick and well-structured response to potential risks.
## The objectives of this project are:
1.To design a real-time safety monitoring system using Spartan-7 FPGA that continuously
observes temperature and smoke levels.2. To simulate and process sensor inputs using digital values for effective testing and
validation of the system.
3. To implement comparator logic that detects when the temperature or smoke exceeds
predefined thresholds.
4. To develop a control mechanism using FSM/comparator that activates alarms and
sprinkler systems automatically during emergency conditions.
5. To synthesize and implement the design on hardware using Xilinx Vivado and verify its
operation through simulation and FPGA testing.

## Block Diagram:
The following block diagram illustrates the major components of the system:
 
<img width="720" height="475" alt="image" src="https://github.com/user-attachments/assets/a2f73940-8214-40cb-b69c-16eff286f5b1" />



## Verilog HDL Code:
The complete Verilog implementation is shown below:
```verilog
`timescale 1ns/1ps
module tb_SafetySystem;
 reg clk;
 reg rst_n;
 reg [7:0] temp_value;
 reg [7:0] smoke_value;
 wire alarm;
 wire sprinkler;
 wire [1:0] status;
 // Instantiate DUT
 SafetySystem dut (
 .clk(clk),
 .rst_n(rst_n),
 .temp_value(temp_value),
 .smoke_value(smoke_value),
 .alarm(alarm),
 .sprinkler(sprinkler),
 .status(status)
 );
 // Clock generation: 10ns period = 100MHz
 always #5 clk = ~clk;
 initial begin
 // Initialize signals
clk = 0;
 rst_n = 0;
 temp_value = 8'd30;
 smoke_value = 8'd40;
 // Apply reset
 $display("Applying Reset...");
 #20;
 rst_n = 1;
 // 1. Normal SAFE condition
 $display("Test 1: Normal SAFE condition");
 #50;
 // 2. Temperature alert
 $display("Test 2: Temperature Alert");
 temp_value = 8'd80;
 #50;
 // 3. Back to SAFE (remove alert)
 $display("Back to SAFE");
 temp_value = 8'd40;
 #50;
 // 4. Smoke alert
 $display("Test 3: Smoke Alert");
 temp_value = 8'd80;
 smoke_value = 8'd90;
 #50;
 // 5. Both alerts
 $display("Test 4: Both Alerts");
temp_value = 8'd90;
 smoke_value = 8'd95;
 #50;
 // 6. Clear both alerts → return to SAFE
 $display("Test 5: Clear Alerts -> SAFE");
 temp_value = 8'd30;
 smoke_value = 8'd20;
 #80;
 $display("Simulation Completed.");
 $finish;
 end
 // Waveform dumping (for GTKWave)
 initial begin
 $dumpfile("SafetySystem.vcd");
 $dumpvars(0, tb_SafetySystem);
 end
endmodule

// ---------------- COMPARATOR ----------------
module fire_detector (
 input wire [7:0] temp_sensor, // Temperature input
 input wire [7:0] smoke_sensor, // Smoke input
 output reg alarm, // Alarm output
 output reg sprinkler // Sprinkler output
);
parameter TEMP_THRESHOLD = 8'd50;
parameter SMOKE_THRESHOLD = 8'd60;
always @(*) begin
 // Default state
 alarm = 0;
 sprinkler = 0;
 // Compare values
 if (temp_sensor >= TEMP_THRESHOLD || smoke_sensor >= SMOKE_THRESHOLD)
 alarm = 1;
 if (temp_sensor >= TEMP_THRESHOLD && smoke_sensor >= SMOKE_THRESHOLD)
 sprinkler = 1;
end
endmodule

// ---------------- SEVEN-SEGMENT DRIVER ----------------
module sevenseg_driver( 
input clk, 
input [7:0] value, 
output reg [6:0] seg, 
output reg [3:0] an 
); 
reg [1:0] digit = 0; 
reg [3:0] bcd; 
always @(posedge clk) begin 
digit <= digit + 1; 
end 
always @(*) begin 
case(digit) 
2'b00: begin an = 4'b1110; bcd = value % 10; end // Units 
2'b01: begin an = 4'b1101; bcd = (value/10) % 10; end // Tens 
2'b10: begin an = 4'b1011; bcd = value / 100; end // Hundreds 
default: begin an = 4'b0111; bcd = 0; end 
endcase 
end 
always @(*) begin 
case(bcd) 
4'd0: seg = 7'b1000000; 
4'd1: seg = 7'b1111001; 
4'd2: seg = 7'b0100100; 
4'd3: seg = 7'b0110000; 
4'd4: seg = 7'b0011001; 
4'd5: seg = 7'b0010010; 
4'd6: seg = 7'b0000010; 
4'd7: seg = 7'b1111000; 
4'd8: seg = 7'b0000000; 
4'd9: seg = 7'b0010000; 
default: seg = 7'b1111111; 
endcase 
end 
endmodule 
```
## TESTBENCH CODE:
```verilog
`timescale 1ns/1ps
module smoke_temp_tb;
// Testbench Signals
reg clk;
reg reset;
reg [7:0] temp_sensor;
reg [7:0] smoke_sensor;
wire alarm;
wire sprinkler;
// Instantiate the DUT (Device Under Test)
smoke_temp_detector dut (
 .clk(clk),
 .reset(reset),
 .temp_sensor(temp_sensor),
 .smoke_sensor(smoke_sensor),
 .alarm(alarm),
 .sprinkler(sprinkler)
);
// Clock generation (10ns period)
always #5 clk = ~clk;
initial begin
 // Initialize signals
 clk = 0;
 reset = 1;
temp_sensor = 8'd0;
smoke_sensor = 8'd0;
 // Reset system
 #10 reset = 0;
 // Test Case 1 – SAFE (No smoke & low temperature)
temp_sensor = 8'd30; smoke_sensor = 8'd20;
 #20;
 // Test Case 2 – ONLY HIGH TEMPERATURE
 temp_sensor = 8'd80; smoke_sensor = 8'd15;
 #20;
 // Test Case 3 – ONLY HIGH SMOKE
 temp_sensor = 8'd40; smoke_sensor = 8'd90;
 #20;
 // Test Case 4 – FIRE CONDITION (Both High)
 temp_sensor = 8'd85; smoke_sensor = 8'd95;
 #20;
// Test Case 5 – Return Back to Safe
 temp_sensor = 8'd35; smoke_sensor = 8'd20;
 #20;
 // Finish simulation
 $stop;
end
// Display Output in Console
initial begin
 $monitor("Time=%0t | Temp=%d | Smoke=%d | Alarm=%b | Sprinkler=%b",
$time, temp_sensor, smoke_sensor, alarm, sprinkler);
end
endmodule
```

## Verilog HDL code for implementation in FPGA:
```verilog

module safety_system_top (
    input  wire clk,               // FPGA clock (100 MHz typical)
    input  wire reset,             // Active high reset
    input  wire [7:0] temp_value,  // Temperature input (sensor / emulator)
    input  wire [7:0] smoke_value, // Smoke level input (sensor / emulator)
    input  wire ack_button,        // User button to stop alarm
    output wire alarm,             // LED/Buzzer output
    output wire sprinkler,         // Sprinkler activation
    output wire [1:0] state_out    // Debug state output
);

    // --- Threshold values (you can modify as needed) ---
    parameter TEMP_THRESHOLD  = 8'd60;   // Temperature > 60 → alert
    parameter SMOKE_THRESHOLD = 8'd40;   // Smoke > 40 → alert

    // --- Comparator Outputs ---
    wire temp_high;
    wire smoke_high;

    // Instantiate comparator module
    comparator_block cmp (
        .temp_value(temp_value),
        .smoke_value(smoke_value),
        .TEMP_TH(TEMP_THRESHOLD),
        .SMOKE_TH(SMOKE_THRESHOLD),
        .temp_high(temp_high),
        .smoke_high(smoke_high)
    );

    // Instantiate FSM
    fire_temp_fsm fsm (
        .clk(clk),
        .reset(reset),
        .temp_high(temp_high),
        .smoke_high(smoke_high),
        .ack_button(ack_button),
        .alarm(alarm),
        .sprinkler(sprinkler),
        .state_out(state_out)
    );

endmodule



// ------------------------------------------------------------
// COMPARATOR
// ------------------------------------------------------------
module comparator_block (
    input  wire [7:0] temp_value,
    input  wire [7:0] smoke_value,
    input  wire [7:0] TEMP_TH,
    input  wire [7:0] SMOKE_TH,
    output reg temp_high,
    output reg smoke_high
);
    always @(*) begin
        temp_high  = (temp_value  > TEMP_TH)  ? 1 : 0;
        smoke_high = (smoke_value > SMOKE_TH) ? 1 : 0;
    end
endmodule


// ------------------------------------------------------------
// FSM CONTROLLER
// ------------------------------------------------------------
module fire_temp_fsm(
    input wire clk,
    input wire reset,
    input wire temp_high,
    input wire smoke_high,
    input wire ack_button,
    output reg alarm,
    output reg sprinkler,
    output reg [1:0] state_out
);

    parameter SAFE    = 2'b00;
    parameter WARNING = 2'b01;
    parameter FIRE    = 2'b10;
    parameter ACK     = 2'b11;

    reg [1:0] state, next_state;

    // State register
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= SAFE;
        else
            state <= next_state;
    end

    // State transition logic
    always @(*) begin
        case (state)
            SAFE:     next_state = (temp_high || smoke_high) ? WARNING : SAFE;
            WARNING:  next_state = (temp_high && smoke_high) ? FIRE :
                                   (!temp_high && !smoke_high) ? SAFE : WARNING;
            FIRE:     next_state = (ack_button) ? ACK : FIRE;
            ACK:      next_state = (!temp_high && !smoke_high) ? SAFE : WARNING;
            default:  next_state = SAFE;
        endcase
    end

    // Output logic
    always @(*) begin
        alarm = (state == WARNING || state == FIRE);
        sprinkler = (state == FIRE || state == ACK);
    end

    always @(*) state_out = state;

endmodule


// ------------------------------------------------------------
// SEVEN SEGMENT DRIVER (with refresh clock)
// ------------------------------------------------------------
module sevenseg_driver(
    input clk,
    input [7:0] value,
    output reg [6:0] seg,
    output reg [3:0] an
);

reg [15:0] refresh_cnt = 0;
reg refresh_clk = 0;

reg [1:0] digit = 0;
reg [3:0] bcd;

// Clock divider ~1 kHz
always @(posedge clk) begin
    refresh_cnt <= refresh_cnt + 1;
    refresh_clk <= refresh_cnt[15];
end

// Digit switching using slow refresh clock
always @(posedge refresh_clk) begin
    digit <= digit + 1;
end

// Select digit and BCD value
always @(*) begin
    case(digit)
        2'b00: begin an = 4'b1110; bcd = value % 10; end
        2'b01: begin an = 4'b1101; bcd = (value / 10) % 10; end
        2'b10: begin an = 4'b1011; bcd = value / 100; end
        default: begin an = 4'b0111; bcd = 0; end
    endcase
end

// BCD to 7-segment
always @(*) begin
    case(bcd)
        4'd0: seg = 7'b1000000;
        4'd1: seg = 7'b1111001;
        4'd2: seg = 7'b0100100;
        4'd3: seg = 7'b0110000;
        4'd4: seg = 7'b0011001;
        4'd5: seg = 7'b0010010;
        4'd6: seg = 7'b0000010;
        4'd7: seg = 7'b1111000;
        4'd8: seg = 7'b0000000;
        4'd9: seg = 7'b0010000;
        default: seg = 7'b1111111;
    endcase
end

endmodule





```
## CONSTRAIN FILE:
```verilog
# clock
set_property -dict {PACKAGE_PIN F14 IOSTANDARD LVCMOS33} [get_ports {clk}]
create_clock -period 10.000 -name gclk [get_ports clk]
# Set Bank 0 voltage / config
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
# Alarm LED -> map to on-board LED[0]
set_property -dict {PACKAGE_PIN G1 IOSTANDARD LVCMOS33} [get_ports {alarm_led}]
# Seven-segment display 0 mapping (use seg[0]..seg[6] -> D0_SEG[0]..D0_SEG[6])
set_property -dict {PACKAGE_PIN D7  IOSTANDARD LVCMOS33} [get_ports {seg[0]}]  # D0_SEG[0]
set_property -dict {PACKAGE_PIN C5  IOSTANDARD LVCMOS33} [get_ports {seg[1]}]  # D0_SEG[1]
set_property -dict {PACKAGE_PIN A5  IOSTANDARD LVCMOS33} [get_ports {seg[2]}]  # D0_SEG[2]
set_property -dict {PACKAGE_PIN B7  IOSTANDARD LVCMOS33} [get_ports {seg[3]}]  # D0_SEG[3]
set_property -dict {PACKAGE_PIN A7  IOSTANDARD LVCMOS33} [get_ports {seg[4]}]  # D0_SEG[4]
set_property -dict {PACKAGE_PIN D6  IOSTANDARD LVCMOS33} [get_ports {seg[5]}]  # D0_SEG[5]
set_property -dict {PACKAGE_PIN B5  IOSTANDARD LVCMOS33} [get_ports {seg[6]}]  # D0_SEG[6]
# Seven-segment anodes (digit enable) mapping: an[3:0] -> D0_AN[0..3]
set_property -dict {PACKAGE_PIN D5  IOSTANDARD LVCMOS33} [get_ports {an[0]}]  # D0_AN[0]
set_property -dict {PACKAGE_PIN C4  IOSTANDARD LVCMOS33} [get_ports {an[1]}]  # D0_AN[1]
set_property -dict {PACKAGE_PIN C7  IOSTANDARD LVCMOS33} [get_ports {an[2]}]  # D0_AN[2]
set_property -dict {PACKAGE_PIN A8  IOSTANDARD LVCMOS33} [get_ports {an[3]}]  # D0_AN[3]
```


## Simulation Results
The waveform shows different states based on sensor values. The system successfully
transitions between SAFE → WARNING → FIRE states. Alarm and sprinkler outputs are
generated according to FSM logic.

 <img width="1215" height="758" alt="image" src="https://github.com/user-attachments/assets/23e47e5c-b730-4f94-86c7-88ad1462c216" />


### Simulation Explanation – (table)
0–100 ns: Temp = 30°C, Smoke = 40% → Alarm OFF, Sprinkler OFF → SAFE

100–200 ns: Temp = 80°C, Smoke = 40% → Alarm ON, Sprinkler OFF → WARNING

200–250 ns: Temp = 90°C, Smoke = 95% → Alarm ON, Sprinkler ON → FIRE

250–300 ns: Temp = 30°C, Smoke = 20% → Alarm OFF, Sprinkler OFF → SAFE


## Hardware Implementation:

The hardware implementation of the project is carried out using the Spartan-7 FPGA (XC7S
series), which enables high-speed and reliable real-time operation for safety-critical
applications.The temperature and smoke sensor values are simulated and fed as digital
inputs to the FPGA for testing. The comparator logic and FSM are synthesized and mapped
onto FPGA resources such as LUTs and flip-flops. The alarm and sprinkler outputs are
assigned to GPIO pins to drive external devices like LEDs or buzzers. 


## Conclusion
The implementation of the temperature and smoke monitoring system using the Spartan-7
FPGA successfully demonstrates a reliable and efficient real-time safety mechanism. By
using hardware-based comparator logic and FSM control, the system ensures quick
response and high accuracy in detecting hazardous conditions. The simulation results in
Vivado verify the correct functioning of the alarm and sprinkler activation logic. 
