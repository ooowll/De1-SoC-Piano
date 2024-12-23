	module m2(
    CLOCK_50, SW, KEY,
	VGA_R, VGA_G, VGA_B,
	VGA_HS, VGA_VS, VGA_BLANK_N, 
	VGA_SYNC_N, VGA_CLK, PS2_CLK, 
    PS2_DAT,  AUD_XCK,        
    AUD_DACDAT, AUD_BCLK, AUD_ADCLRCK,    
    AUD_DACLRCK, FPGA_I2C_SCLK, FPGA_I2C_SDAT,LEDR);
	
	input CLOCK_50;	
	input [7:0] SW;
	input [3:0] KEY;

    //VGA
	output [7:0] VGA_R, VGA_G, VGA_B;
	output VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK;	

    //PS2
	inout PS2_CLK, PS2_DAT;

    //Audio
	output AUD_XCK, AUD_DACDAT;    
    inout AUD_BCLK, AUD_ADCLRCK, AUD_DACLRCK;     
    output FPGA_I2C_SCLK;   
    inout FPGA_I2C_SDAT;
	output [9:0]LEDR;
	

	
	wire [7:0] letterCode;
    wire keyPressed;
    wire [12:0] encodedLetter;
    wire [12:0] LiveEncodedLetter;

    assign encodedLetter = LiveEncodedLetter;
    
	 PS2_Controller PS2(
        .CLOCK_50(CLOCK_50),
        .reset(~KEY[0]),
        .PS2_CLK(PS2_CLK),
        .PS2_DAT(PS2_DAT),
        .received_data(letterCode),
        .received_data_en(keyPressed)
    );
    
    wire [12:0] active_letter; 

    assign active_letter = LiveEncodedLetter | recordedLetter;

    wire [31:0] tone0 ; //active playing
    wire [31:0] tone1 ;
    wire [31:0] tone2 ;
    wire [31:0] tone3 ;
    wire [31:0] tone4 ;
    wire [31:0] tone5 ;
    wire [31:0] tone6 ;
    wire [31:0] tone7 ;
    wire [31:0] tone8 ;
    wire [31:0] tone9 ;
    wire [31:0] tone10 ;
    wire [31:0] tone11;
    wire [31:0] tone12 ;
	wire [31:0] recordingTone; //recorded playback
  integer i;
  integer j;
  integer k;



// sound output 
    parameter CLOCK_FREQ = 50000000;
	
    reg [31:0] delay_cnt [12:0];
    reg square_wave [12:0];
    reg [31:0] half_period [12:0];
    

   KeyboardTone u1 (.active_letter(active_letter), .SW(SW[0]),
   .tone0(tone0),
   .tone1(tone1),
   .tone2(tone2),
   .tone3(tone3),
   .tone4(tone4),
   .tone5(tone5),
   .tone6(tone6),
   .tone7(tone7),
   .tone8(tone8),
   .tone9(tone9),
   .tone10(tone10),
   .tone11(tone11),
   .tone12(tone12)
   ); //assigns the tone 

 


   reg [31:0] tones [12:0];

  always @ (*) begin 
    tones[0] = tone0;
    tones[1] = tone1;
    tones[2] = tone2;
    tones[3] = tone3;
    tones[4] = tone4;
    tones[5] = tone5;
    tones[6] = tone6;
    tones[7] = tone7;
    tones[8] = tone8;
    tones[9] = tone9;
    tones[10] = tone10;
    tones[11] = tone11;
    tones[12] = tone12;
    
   end 
    //create a half period for each note 

   

    always @(*) begin
       
        for (i=0; i<13; i = i+1 ) begin
        if (tones[i] != 0)
            half_period[i] = (CLOCK_FREQ / (2 * tones[i]));
        else
            half_period[i] = 0;
        end
    end


    always @(posedge CLOCK_50) begin
         
        for (j=0; j<13; j = j+1 ) begin
    if (active_letter[j]) begin //if note is active, only want a period to be generated
    //if the note is actually being pressed down. else the period should not exist and note should not play
        if (delay_cnt[j] >= half_period[j]) begin
            delay_cnt[j] <= 0;
            square_wave[j] <= ~square_wave[j];
        end else begin
            delay_cnt[j] <= delay_cnt[j] + 1;
        end
    end
     else begin  //if note is not active
        delay_cnt[j] <= 0;
        square_wave[j] <= 0;
    end
    end
    end

    //need to scale amplitude since multiple notes will make the note louder 
    // create a summed square wave to output 

    parameter MAX_AMPLITUDE = 32'h2625A00;
    reg signed [31:0] amplitude;
always @(*) begin
    if (num_of_notes > 0)
        amplitude = MAX_AMPLITUDE / num_of_notes;
    else
        amplitude = 0;
end

    reg signed [31:0] output_wave;
    always @(*) begin 
    output_wave = 0;
   
    for (k=0; k<13; k = k+1 ) begin
        if (active_letter[k]) begin
            output_wave = output_wave + (square_wave[k] ? amplitude : -amplitude);
        end
    end
end

    integer num_of_notes;
    always @(*) begin 
        num_of_notes = active_letter[0]+active_letter[1]+active_letter[2]+active_letter[3]+active_letter[4]
        +active_letter[5]+active_letter[6]+active_letter[7]+active_letter[8]+active_letter[9]
        +active_letter[10]+active_letter[11]+active_letter[12];
    end



    wire signed [31:0] audio_out = output_wave;//audio is outputted here

    Audio_Controller Audio_Controller (
        .CLOCK_50(CLOCK_50),
        .reset(~KEY[0]),
        .clear_audio_in_memory(),
        .clear_audio_out_memory(),
        .left_channel_audio_out(audio_out),
        .right_channel_audio_out(audio_out),
        .write_audio_out(1'b1),
        .AUD_ADCDAT(1'b0),
        .AUD_BCLK(AUD_BCLK),
        .AUD_ADCLRCK(AUD_ADCLRCK),
        .AUD_DACLRCK(AUD_DACLRCK),
        .audio_in_available(),
        .audio_out_allowed(),
        .AUD_XCK(AUD_XCK),
        .AUD_DACDAT(AUD_DACDAT)
    );

    avconf #(.USE_MIC_INPUT(1)) avc (
        .FPGA_I2C_SCLK(FPGA_I2C_SCLK),
        .FPGA_I2C_SDAT(FPGA_I2C_SDAT),
        .CLOCK_50(CLOCK_50),
        .reset(~KEY[0])
    );

reg [7:0] X;      
reg [6:0] Y;
wire [7:0]white1XC;//14x59
wire [6:0]white1YC;  
wire [7:0]white2XC;//10x59
wire [6:0]white2YC;         
wire [7:0]white3XC;//18x59
wire [6:0]white3YC;  
wire [2:0]blackXC;//8x58
wire [5:0]blackYC;
wire [7:0]bkgXC;//160x120
wire [6:0]bkgYC;

wire [11:0]addressWhite1Key = (white1YC*14) + white1XC;//0 to 2124 pixels
wire [11:0]addressWhite2Key = (white2YC*10) + white2XC;//0 to 2124 pixels
wire [11:0]addressWhite3Key = (white3YC*18) + white3XC;//0 to 2124 pixels
wire [8:0]addressBlackKey = (blackYC*8) + blackXC;//0 to 464 pixels
wire [14:0]addressBKG = (bkgYC*160) + bkgXC;
reg [4:0]vgaState, nextvgastate;
wire[2:0] white1, white2, white3, blackKey, blank;//colour
wire[2:0] displayKey; //colourout


// keyleftrom kl1(addressWhiteKey, CLOCK_50, leftKey);
// keymiddlerom km1(addressWhiteKey, CLOCK_50, middleKey);
// keyrightrom kr1(addressWhiteKey, CLOCK_50, rightKey);
// keywhiterom kw1(addressWhiteKey, CLOCK_50, whiteKey);
 keyblackrom kb1(addressBlackKey, CLOCK_50, blackKey);
 white1rom w1r(addressWhite1Key, CLOCK_50, white1);
 white2rom w2r(addressWhite2Key, CLOCK_50, white2);
 white3rom w3r(addressWhite3Key, CLOCK_50, white3);


xycounter14x59 wcount1(CLOCK_50, KEY[0], (vgaState == keyC4a && active_letter[12]|| vgaState == keyE4a  && active_letter[8]|| vgaState == keyF4a && active_letter[7] || vgaState == keyB4a && active_letter[1]), white1XC, white1YC);
xycounter10x59 wcount2(CLOCK_50, KEY[0], (vgaState == keyD4a  && active_letter[10]|| vgaState == keyG4a  && active_letter[5]|| vgaState == keyA4a && active_letter[3]), white2XC, white2YC);
xycounter18x59 wcount3(CLOCK_50, KEY[0], (vgaState == keyC4b  && active_letter[12]|| vgaState == keyD4b  && active_letter[10]|| vgaState == keyE4b  && active_letter[8]|| vgaState == keyF4b  && active_letter[7]||   vgaState == keyG4b  && active_letter[5]|| vgaState == keyA4b  && active_letter[3]|| vgaState == keyB4b  && active_letter[1]|| vgaState == keyC5a && active_letter[0] || vgaState == keyC5b && active_letter[0]), white3XC, white3YC);
xycounterblack count2(CLOCK_50, KEY[0], (vgaState == keyCh4  && active_letter[11]|| vgaState == keyDh4 && active_letter[9] || vgaState == keyFh4 && active_letter[6] || vgaState == keyGh4 && active_letter[4] || vgaState == keyAh4 && active_letter[2]), blackXC, blackYC);
xycounterbkg count3(CLOCK_50, KEY[0], vgaState == none, bkgXC, bkgYC);
parameter[4:0] none = 5'b00000, keyC4a = 5'b00001, keyC4b = 5'b00010, 
keyCh4 = 5'b00011, keyD4a = 5'b00100, keyD4b = 5'b00101, keyDh4 = 5'b00110, 
keyE4a = 5'b00111, keyE4b = 5'b01000, keyF4a = 5'b01001, keyF4b = 5'b01010, 
keyFh4 = 5'b01011, keyG4a = 5'b01100, keyG4b = 5'b01101, keyGh4 = 5'b01110, 
keyA4a = 5'b01111, keyA4b = 5'b10000, keyAh4 = 5'b10001, keyB4a = 5'b10010, 
keyB4b = 5'b10011, keyC5a = 5'b10100, keyC5b = 5'b10101;

always@(*)
begin
    case(vgaState)
    none:
    begin
    if(bkgXC == 8'd159 && bkgYC == 7'd119) 
        begin
        if(active_letter[12]) nextvgastate = keyC4a;
        else if(active_letter[11]) nextvgastate = keyCh4;
        else if(active_letter[10]) nextvgastate = keyD4a;
        else if(active_letter[9]) nextvgastate = keyDh4;
        else if(active_letter[8]) nextvgastate = keyE4a;
        else if(active_letter[7]) nextvgastate = keyF4a;
        else if(active_letter[6]) nextvgastate = keyFh4;
        else if(active_letter[5]) nextvgastate = keyG4a;
        else if(active_letter[4]) nextvgastate = keyGh4;
        else if(active_letter[3]) nextvgastate = keyA4a;
        else if(active_letter[2]) nextvgastate = keyAh4;
        else if(active_letter[1]) nextvgastate = keyB4a;
        else if(active_letter[0]) nextvgastate = keyC5a;
        
        end
    else nextvgastate = none;
    end
    
    keyC4a:
    begin
    if(active_letter[12])
    begin
        if(white1XC == 5'd13 && white1YC == 7'd58) nextvgastate = keyC4b;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyC4a;  
    end
    else nextvgastate = keyC4b;
    end

    keyC4b:
    begin
    if(active_letter[12])
    begin
        if(white3XC == 5'd17 && white3YC == 7'd58) nextvgastate = keyCh4;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyC4b;  
    end
    else nextvgastate = keyCh4;
    end

    keyCh4:
    begin
    if(active_letter[11])
    begin
        if(blackXC == 5'd7 && blackYC == 7'd58) nextvgastate = keyD4a;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyCh4;
    end
    else nextvgastate = keyD4a;
    end

    keyD4a:
    begin
    if(active_letter[10])
    begin
        if(white2XC == 5'd9 && white2YC == 7'd58) nextvgastate = keyD4b;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyD4a;
    end
    else nextvgastate = keyD4b;
    end

    keyD4b:
    begin
    if(active_letter[10])
    begin
        if(white3XC == 5'd17 && white3YC == 7'd58) nextvgastate = keyDh4;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyD4b;
    end
    else nextvgastate = keyDh4;
    end
 
    keyDh4:
    begin
    if(active_letter[9])
    begin
        if(blackXC == 5'd7 && blackYC == 7'd58) nextvgastate = keyE4a;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyDh4;
    end
    else nextvgastate = keyE4a;
    end

    keyE4a:
    begin
    if(active_letter[8])
    begin
        if(white1XC == 5'd13 && white1YC == 7'd58) nextvgastate = keyE4b;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyE4a;
    end
    else nextvgastate = keyE4b;
    end

    keyE4b:
    begin
    if(active_letter[8])
    begin
        if(white3XC == 5'd17 && white3YC == 7'd58) nextvgastate = keyF4a;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyE4b;
    end
    else nextvgastate = keyF4a;
    end

    keyF4a:
    begin
    if(active_letter[7])
    begin
        if(white1XC == 5'd13 && white1YC == 7'd58) nextvgastate = keyF4b;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyF4a;
    end
    else nextvgastate = keyF4b;
    end

    keyF4b:
    begin
    if(active_letter[7])
    begin
        if(white3XC == 5'd17 && white3YC == 7'd58) nextvgastate = keyFh4;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyF4b;
    end
    else nextvgastate = keyFh4;
    end

    keyFh4:
    begin
    if(active_letter[6])
    begin
    if(blackXC == 5'd7 && blackYC == 7'd58) nextvgastate = keyG4a;
    else if (letterCode == 8'hF0)nextvgastate = none;
    else nextvgastate = keyFh4;
    end
    else nextvgastate = keyG4a;
    end

    keyG4a:
    begin
    if(active_letter[5])
    begin
        if(white2XC == 5'd9 && white2YC == 7'd58) nextvgastate = keyG4b;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyG4a;
    end
    else nextvgastate = keyG4b;
    end

    keyG4b:
    begin
    if(active_letter[5])
    begin
        if(white3XC == 5'd17 && white3YC == 7'd58) nextvgastate = keyGh4;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyG4b;
    end
    else nextvgastate = keyGh4;
    end

    keyGh4:
    begin
    if(active_letter[4])
    begin
    if(blackXC == 5'd7 && blackYC == 7'd58) nextvgastate = keyA4a;
    else if (letterCode == 8'hF0)nextvgastate = none;
    else nextvgastate = keyGh4;
    end
    else nextvgastate = keyA4a;
    end

    keyA4a:
    begin
    if(active_letter[3])
    begin
        if(white2XC == 5'd9 && white2YC == 7'd58) nextvgastate = keyA4b;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyA4a;
    end
    else nextvgastate = keyA4b;
    end

    keyA4b:
    begin
    if(active_letter[3])
    begin
        if(white3XC == 5'd17 && white3YC == 7'd58) nextvgastate = keyAh4;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyA4b;
    end
    else nextvgastate = keyAh4;
    end

    keyAh4:
    begin
    if(active_letter[2])
    begin
    if(blackXC == 5'd7 && blackYC == 7'd58) nextvgastate = keyB4a;
    else if(letterCode == 8'hF0) nextvgastate = none;
    else nextvgastate = keyAh4;
    end
    else nextvgastate = keyB4a;
    end

    keyB4a:
    begin
    if(active_letter[1])
    begin
        if(white1XC == 5'd13 && white1YC == 7'd58) nextvgastate = keyB4b;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyB4a;
    end
    else nextvgastate = keyB4b;
    end

    keyB4b:
    begin
    if(active_letter[1])
    begin
        if(white3XC == 5'd17 && white3YC == 7'd58) nextvgastate = keyC5a;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyB4b;
    end
    else nextvgastate = keyC5a;
    end

    keyC5a:
    begin
    if(active_letter[0])
    begin
        if(white3XC == 5'd17 && white3YC == 7'd58) nextvgastate = keyC5b;
        else if (letterCode == 8'hF0)nextvgastate = none;
        else nextvgastate = keyC5a;
    end
    else nextvgastate = keyC5b;
    end

    keyC5b:
    begin
    if(active_letter[0])
    begin
        if((white3XC == 5'd17 && white3YC == 7'd58) && active_letter != 13'b0) nextvgastate = none;
        else if (!active_letter[0])nextvgastate = none;
        else nextvgastate = keyC5b;
    end
    else nextvgastate = none;
    end
    endcase
end

assign LEDR[9:0] = active_letter[9:0];

always@(posedge CLOCK_50)
    if(KEY[0] == 0)
        vgaState <= none;
    else 
        vgaState <= nextvgastate;

always@(*)
begin
    case(vgaState)
    none:
    if(active_letter == 13'b0)
    begin
    X<=bkgXC-1;
    Y<=bkgYC;
    end
    keyC4a:
    if(active_letter[12])
    begin
    X<=1+white1XC;
    Y<=1+white1YC;
    end
    keyC4b:
    if(active_letter[12])
    begin
    X<=1+white3XC;
    Y<=60+white3YC;
    end
    keyCh4:
    if(active_letter[11])
    begin
    X<=16+blackXC;
    Y<=blackYC;
    end
    keyD4a:
    if(active_letter[10])
    begin
    X<=25+white2XC;
    Y<=1+white2YC;
    end
    keyD4b:
    if(active_letter[10])
    begin
    X<=21+white3XC;
    Y<=60+white3YC;
    end
    keyDh4:
    if(active_letter[9])
    begin
    X<=36+blackXC;
    Y<=blackYC;
    end
    keyE4a:
    if(active_letter[8])
    begin
    X<=45+white1XC;
    Y<=1+white1YC;
    end
    keyE4b:
    if(active_letter[8])
    begin
    X<=41+white3XC;
    Y<=60+white3YC;
    end
    keyF4a:
    if(active_letter[7])
    begin
    X<=61+white1XC;
    Y<=1+white1YC;
    end
    keyF4b:
    if(active_letter[7])
    begin
    X<=61+white3XC;
    Y<=60+white3YC;
    end
    keyFh4:
    if(active_letter[6])
    begin
    X<=76+blackXC;
    Y<=blackYC;
    end
    keyG4a:
    if(active_letter[5])
    begin
    X<=85+white2XC;
    Y<=1+white2YC;
    end
    keyG4b:
    if(active_letter[5])
    begin
    X<=81+white3XC;
    Y<=60+white3YC;
    end
    keyGh4:
    if(active_letter[4])
    begin
    X<=96+blackXC;
    Y<=blackYC;
    end
    keyA4a:
    if(active_letter[3])
    begin
    X<=105+white2XC;
    Y<=1+white2YC;
    end
    keyA4b:
    if(active_letter[3])
    begin
    X<=101+white3XC;
    Y<=60+white3YC;
    end
    keyAh4:
    if(active_letter[2])
    begin
    X<=116+blackXC;
    Y<=blackYC;
    end
    keyB4a:
    if(active_letter[1])
    begin
    X<=125+white1XC;
    Y<=1+white1YC;
    end
    keyB4b:
    if(active_letter[1])
    begin
    X<=121+white3XC;
    Y<=60+white3YC;
    end
    keyC5a:
    if(active_letter[0])
    begin
    X<=141+white3XC;
    Y<=1+white3YC;
    end
    keyC5b:
    if(active_letter[0])
    begin
    X<=141+white3XC;
    Y<=60+white3YC;
    end
    endcase
end

// wire [2:0]C4, Ch4, D4, Dh4, E4, F4, Fh4, G4, Gh4, A4, Ah4, B4, C5, blank, displayNote;
// romc4 r1(address, CLOCK_50, C4);
// romch4 r2(address, CLOCK_50, Ch4);
// romd4 r3(address, CLOCK_50, D4);
// romdh4 r4(address, CLOCK_50, Dh4);
// rome4 r5(address, CLOCK_50, E4);
// romf4 r6(address, CLOCK_50, F4);
// romfh4 r7(address, CLOCK_50, Fh4);
// romg4 r8(address, CLOCK_50, G4);
// romgh4 r9(address, CLOCK_50, Gh4);
// roma4 r10(address, CLOCK_50, A4);
// romah4 r11(address, CLOCK_50, Ah4);
// romb4 r12(address, CLOCK_50, B4);
// romc5 r13(address, CLOCK_50, C5);
romblank r14(addressBKG, CLOCK_50, blank);
// mux13to1 u5(C4, Ch4, D4, Dh4, E4, F4, Fh4, G4, Gh4, A4, Ah4, B4, C5, blank, active_letter, displayNote);
mux6to1 m5t1(white1, white2, white3, blackKey, blank, vgaState, displayKey);



vga_adapter VGA (
.resetn(KEY[0]),
.clock(CLOCK_50),
.colour(displayKey),
.x(X),
.y(Y),
.plot(1'b1),
.VGA_R(VGA_R),
.VGA_G(VGA_G),
.VGA_B(VGA_B),
.VGA_HS(VGA_HS),
.VGA_VS(VGA_VS),
.VGA_BLANK_N(VGA_BLANK_N),
.VGA_SYNC_N(VGA_SYNC_N),
.VGA_CLK(VGA_CLK));
defparam VGA.RESOLUTION = "160x120";
defparam VGA.MONOCHROME = "FALSE";
defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
//defparam VGA.BACKGROUND_IMAGE = "blank.mif";


reg writing;
reg playing;
wire write;
wire play;
wire [9:0]recordingAddress;
wire [12:0]recordedLetter;
wire [12:0]recordingOutput;
wire [18:0]slowClock;
parameter [1:0]recordingOff = 2'b0, recordingOn = 2'b01, playback = 2'b10;
reg [1:0]recordingState;

always@(posedge CLOCK_50)
begin
if(!KEY[0]) 
    begin
        recordingState <= recordingOff;
        writing <= 0;
        playing <= 0;
    end

case(recordingState)
recordingOff:
begin
if(write) 
    begin
        recordingState <= recordingOn;
        writing <= 1;
    end
else if(play)
    begin
        recordingState <= playback;
        playing <= 1;
    end
else 
    begin
        recordingState <= recordingOff;
        writing <= 0;
        playing <= 0;
    end
end

recordingOn:
if(recordingAddress == 10'd1000) 
begin
recordingState <= recordingOff;
writing <=0;
end
else 
begin
    recordingState <= recordingOn;
end

playback:
if(recordingAddress == 10'd1000)
begin
    recordingState <= recordingOff;
    playing <= 0;
end
else 
begin
    recordingState <= playback;
    playing <= 1;
end
endcase
end

recordingram rr1(recordingAddress, CLOCK_50, encodedLetter, writing, recordingOutput);
clock19 c19(CLOCK_50, slowClock, KEY[0]);
count1000 c1000(CLOCK_50, slowClock, writing, playing, KEY[0], recordingAddress);
playRecording pr1(CLOCK_50, playing, recordingOutput, recordedLetter);
keyEncoder ke1(CLOCK_50, letterCode, keyPressed, KEY[0], LiveEncodedLetter, write, play);
endmodule



module playRecording(input clock, input playing, input [12:0]recordingOutput, output reg [12:0]recordedLetter);
always@(posedge clock) begin
if(playing == 1)
recordedLetter[12:0] <= recordingOutput[12:0];
else 
recordedLetter <= 13'b0;
end
endmodule

module clock19(input clock, output reg[18:0]slowClock, input resetn);
always @ (posedge clock)
begin
    if (resetn == 0)
        slowClock <= 19'b0;
    else if (slowClock > 19'd500000)
        slowClock <= 19'b0;
    else 
        slowClock <= slowClock+1;
end
endmodule

module count1000(clock, slowClock, writing, playing, resetn, Q);
input clock; 
input [18:0]slowClock;
input writing;
input playing;
input resetn;
output reg[9:0] Q;

always @ (posedge clock)
begin
    if (resetn == 0 || (writing == 0 && playing == 0))
        Q <= 10'b0;
    else if (slowClock == 19'd500000)
        Q <= Q+1;

    if(Q > 10'd1001)
        Q <= 10'b0;
end
endmodule

//module that assigns the tone
module KeyboardTone (input [12:0] active_letter, input SW,
output reg [31:0] tone0,
output reg [31:0] tone1,
output reg [31:0] tone2,
output reg [31:0] tone3,
output reg [31:0] tone4,
output reg [31:0] tone5,
output reg [31:0] tone6,
output reg [31:0] tone7,
output reg [31:0] tone8,
output reg [31:0] tone9,
output reg [31:0] tone10,
output reg [31:0] tone11,
output reg [31:0] tone12);

parameter C4 = 261, Ch4 = 277, D4 = 293, Dh4 = 311, E4 = 329, 
F4 = 349, Fh4 = 370, G4 = 392,  Gh4 = 415, A4 = 440, Ah4 = 466, B4 = 493, C5 = 522;

//need to create a sum of tones

always @(*) 
begin
 
//tones is now an array, final outputted tone is sum of each tone. If the corrresponding
//encoded letter is true, then the note will be active
tone0 = SW ? ((active_letter[0]) ? C5 : 0 ) : ((active_letter[0]) ? C5/2 : 0 );
tone1 = SW ? ((active_letter[1]) ? B4 : 0) : ((active_letter[1]) ? B4/2 : 0 ); 
tone2 = SW ? ((active_letter[2]) ? Ah4 : 0) : ((active_letter[2]) ? Ah4/2 : 0); 
tone3 =  SW ? ((active_letter[3]) ? A4 : 0) : ((active_letter[3]) ? A4 /2 : 0);
tone4 =  SW ? ((active_letter[4]) ? Gh4 : 0) : ((active_letter[4]) ? Gh4/2 : 0);
tone5 =  SW ? ((active_letter[5]) ? G4 : 0) : ((active_letter[5]) ? G4 /2 : 0);
tone6 =  SW ? ((active_letter[6]) ? Fh4 : 0) : ((active_letter[6]) ? Fh4/2 : 0);
tone7 =  SW ? ((active_letter[7]) ? F4 : 0) : ((active_letter[7]) ? F4/2 : 0);
tone8 =  SW ? ((active_letter[8]) ? E4 : 0) : ((active_letter[8]) ? E4/2 : 0);
tone9 =  SW ? ((active_letter[9]) ? Dh4: 0) : ((active_letter[9]) ? Dh4/2 : 0);
tone10 = SW ? ((active_letter[10]) ? D4 : 0) : ((active_letter[10]) ? D4/2 : 0);
tone11 = SW ? ((active_letter[11]) ? Ch4 : 0) : ((active_letter[11]) ? Ch4/2 : 0);
tone12 = SW ? ((active_letter[12]) ? C4 : 0) : ((active_letter[12]) ? C4/2 : 0);      

// outputted is each indiviudal tone
end

endmodule




module mux13to1(input [2:0]C4, input [2:0]Ch4, input [2:0]D4, input [2:0]Dh4, 
input [2:0]E4, input [2:0]F4, input [2:0]Fh4, input [2:0]G4, input [2:0]Gh4, 
input [2:0]A4, input [2:0]Ah4, input [2:0]B4, input [2:0]C5, input [2:0]blank, 
input [12:0]active_letter, output reg [2:0]note);
always@(*)
begin
    if(active_letter[12]) note <= C4;
    else if(active_letter[11]) note <= Ch4;
    else if(active_letter[10]) note <= D4;
    else if(active_letter[9]) note <= Dh4;
    else if(active_letter[8]) note <= E4;
    else if(active_letter[7]) note <= F4;
    else if(active_letter[6]) note <= Fh4;
    else if(active_letter[5]) note <= G4;
    else if(active_letter[4]) note <= Gh4;
    else if(active_letter[3]) note <= A4;
    else if(active_letter[2]) note <= Ah4;
    else if(active_letter[1]) note <= B4;
    else if(active_letter[0]) note <= C5;
    else note <= blank;
end
endmodule

module mux6to1(input [2:0]white1, input [2:0]white2, input [2:0]white3, input [2:0]blackKey, input[2:0]blank, input [4:0]vgaState, output reg [2:0]displayKey);
parameter[4:0] none = 5'b00000, keyC4a = 5'b00001, keyC4b = 5'b00010, 
keyCh4 = 5'b00011, keyD4a = 5'b00100, keyD4b = 5'b00101, keyDh4 = 5'b00110, 
keyE4a = 5'b00111, keyE4b = 5'b01000, keyF4a = 5'b01001, keyF4b = 5'b01010, 
keyFh4 = 5'b01011, keyG4a = 5'b01100, keyG4b = 5'b01101, keyGh4 = 5'b01110, 
keyA4a = 5'b01111, keyA4b = 5'b10000, keyAh4 = 5'b10001, keyB4a = 5'b10010, 
keyB4b = 5'b10011, keyC5a = 5'b10100, keyC5b = 5'b10101;
always@(*)
begin
    if(vgaState == keyC4a) displayKey <= white1;
    else if(vgaState == keyC4b) displayKey <= white3;
    else if(vgaState == keyCh4) displayKey <= blackKey;
    else if(vgaState == keyD4a) displayKey <= white2;
    else if(vgaState == keyD4b) displayKey <= white3;
    else if(vgaState == keyDh4) displayKey <= blackKey;
    else if(vgaState == keyE4a) displayKey <= white1;
    else if(vgaState == keyE4b) displayKey <= white3;
    else if(vgaState == keyF4a) displayKey <= white1;
    else if(vgaState == keyF4b) displayKey <= white3;
    else if(vgaState == keyFh4) displayKey <= blackKey;
    else if(vgaState == keyG4a) displayKey <= white2;
    else if(vgaState == keyG4b) displayKey <= white3;
    else if(vgaState == keyGh4) displayKey <= blackKey;
    else if(vgaState == keyA4a) displayKey <= white2;
    else if(vgaState == keyA4b) displayKey <= white3;
    else if(vgaState == keyAh4) displayKey <= blackKey;
    else if(vgaState == keyB4a) displayKey <= white1;
    else if(vgaState == keyB4b) displayKey <= white3;
    else if(vgaState == keyC5a) displayKey <= white3;
    else if(vgaState == keyC5b) displayKey <= white3;
    else if(vgaState == none)displayKey <= blank;
end
endmodule



module xycounterbkg(input clock, input resetn, input e, output reg [7:0]X, output reg [6:0]Y);
always @ (posedge clock)
begin
if (resetn == 0 || e == 0)
	begin   
        X <=0;
        Y <= 0;
	end
else 
    if (X > 8'd159)
        begin
            Y <= Y+1'b1;
            X <= 0;
        end
    if(Y > 7'd119 && X > 8'd159)
        begin
            X <= 0;
            Y <= 0;
        end
 X <= X+1'b1;

end
endmodule


module keyEncoder(clock, letterCode, keyPressed, resetn, LiveEncodedLetter, write, play);
	input clock;
    input [7:0]letterCode;
    input keyPressed, resetn;
    output reg [12:0]LiveEncodedLetter;
    output reg write, play;
    reg state;
    parameter idle = 1'b0, break = 1'b1;
	always @(posedge clock)
    begin
        if (!resetn)
        begin
            state <= idle;
            LiveEncodedLetter <= 13'b0;  
        end
	else if (keyPressed)
	    begin
	    case (state)
	        idle:
	        if (letterCode == 8'hF0) state <= break;
	        else
	            begin
                     if(letterCode == 8'h15) LiveEncodedLetter[12] <= 1;
                     if(letterCode == 8'h1E) LiveEncodedLetter[11] <=1;
                     if(letterCode == 8'h1D) LiveEncodedLetter[10] <= 1;
                     if(letterCode == 8'h26) LiveEncodedLetter[9] <= 1;
                     if(letterCode == 8'h24) LiveEncodedLetter[8] <= 1;
                     if(letterCode == 8'h2D) LiveEncodedLetter[7] <= 1;
                     if(letterCode == 8'h2E) LiveEncodedLetter[6] <= 1;
                     if(letterCode == 8'h2C) LiveEncodedLetter[5] <= 1;
                     if(letterCode == 8'h36) LiveEncodedLetter[4] <= 1;
                     if(letterCode == 8'h35) LiveEncodedLetter[3] <= 1;
                     if(letterCode == 8'h3D) LiveEncodedLetter[2] <= 1;
                     if(letterCode == 8'h3C) LiveEncodedLetter[1] <= 1;
                     if(letterCode == 8'h43) LiveEncodedLetter[0] <= 1;
                     if(letterCode == 8'h44) write <= 1;
                     if(letterCode == 8'h4D) play <= 1;
		            state <= idle;
	            end
	        break:
	        begin
                    if(letterCode == 8'h15) LiveEncodedLetter[12] <= 0;
                     if(letterCode == 8'h1E) LiveEncodedLetter[11] <=0;
                     if(letterCode == 8'h1D) LiveEncodedLetter[10] <= 0;
                     if(letterCode == 8'h26) LiveEncodedLetter[9] <= 0;
                     if(letterCode == 8'h24) LiveEncodedLetter[8] <= 0;
                     if(letterCode == 8'h2D) LiveEncodedLetter[7] <= 0;
                     if(letterCode == 8'h2E) LiveEncodedLetter[6] <= 0;
                     if(letterCode == 8'h2C) LiveEncodedLetter[5] <= 0;
                     if(letterCode == 8'h36) LiveEncodedLetter[4] <= 0;
                     if(letterCode == 8'h35) LiveEncodedLetter[3] <= 0;
                     if(letterCode == 8'h3D) LiveEncodedLetter[2] <= 0;
                     if(letterCode == 8'h3C) LiveEncodedLetter[1] <= 0;
                    if(letterCode == 8'h43) LiveEncodedLetter[0] <= 0;
                    if(letterCode == 8'h44) write <= 0;
                     if(letterCode == 8'h4D) play <= 0;
                 state <= idle;
	        end
	    endcase
	    end
	end
endmodule

module count (Clock, Resetn, E, Q);
    parameter n = 8;
    input Clock, Resetn, E;
    output reg [n-1:0] Q;

    always @ (posedge Clock)
        if (Resetn == 0 || E == 0)
            Q <= 0;
        else if (E)
                Q <= Q + 1;
endmodule

module xycounter18x59(input clock, input resetn, input e, output reg [7:0]X, output reg [6:0]Y);
always @ (posedge clock)
begin
if (resetn == 0 || e == 0)
	begin   
        X <=0;
        Y <= 0;
	end
else 
     if(Y >= 7'd58 && X >= 8'd17)
        begin
            X <= 0;
            Y <= 0;
        end
    else if (X >= 8'd17)
        begin
            Y <= Y+1'b1;
            X <= 0;
        end

 else X <= X+1'b1;

end
endmodule
module xycounter14x59(input clock, input resetn, input e, output reg [7:0]X, output reg [6:0]Y);
always @ (posedge clock)
begin
if (resetn == 0 || e == 0)
	begin   
        X <=0;
        Y <= 0;
	end
else 
     if(Y >= 7'd58 && X >= 8'd13)
        begin
            X <= 0;
            Y <= 0;
        end
   else if (X >= 8'd13)
        begin
            Y <= Y+1'b1;
            X <= 0;
        end

 else X <= X+1'b1;

end
endmodule
module xycounter10x59(input clock, input resetn, input e, output reg [7:0]X, output reg [6:0]Y);
always @ (posedge clock)
begin
if (resetn == 0 || e == 0)
	begin   
        X <=0;
        Y <= 0;
	end
else 
     if(Y == 7'd58 && X == 8'd9)
        begin
            X <= 0;
            Y <= 0;
        end
    else if (X == 8'd9)
        begin
            Y <= Y+1'b1;
            X <= 0;
        end

 else X <= X+1'b1;

end
endmodule
module xycounterblack(input clock, input resetn, input e, output reg [4:0]X, output reg [6:0]Y);
always @ (posedge clock)
begin
if (resetn == 0 || e == 0)
	begin
        X <=0;
        Y <= 0;
	end
else
 if(Y == 10'd58 && X == 10'd8)
	begin
        X <= 0;
        Y <= 0;
	end
else if (X == 10'd8)
	begin
        Y <= Y+1'b1;
        X <= 0;
	end

else X <= X+1'b1;
end
endmodule