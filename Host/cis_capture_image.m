clear; clc; close all;
%% CIS Sensor Host Controller  -  STM32H723 capture board (MATLAB)
%  Line-scan IMAGE capture: grabs NUM_LINES lines and stacks them into a
%  2-D image (rows = successive captures, columns = pixels along the bar).
%
%  NOTE: a CIS is a LINE sensor - the target (or the sensor) must move
%  between captures, otherwise every row is the same and the "image" is
%  just the same line repeated.
%
% Protocol (matches the firmware):
%    's'            -> capture one line; board returns 2*ADC_BUF_LEN*2 bytes
%    'r'/'g'/'b'/'w'-> LED color on
%    'o'            -> LED off
%    'd' + uint16   -> set trigger width in WHOLE CIS CLK cycles (LE)
%
% Wiring recap:
%    ADC1 (PF11) = AN_1 = sensor half 1
%    ADC2 (PF13) = AN_2 = sensor half 2
%    Full line = 20400 px, 10200 per half.
%
% Requires the serialport() interface (MATLAB R2019b+).

%% ---- Configuration -------------------------------------------------------
PORT           = "COM4";        % Windows: "COM7" | Linux: "/dev/ttyACM0"
ADC_BUF_LEN    = 10200;         % samples per channel (must match firmware)
BYTES_PER_HALF = ADC_BUF_LEN*2; % uint16
TOTAL_BYTES    = 2*BYTES_PER_HALF;
READ_TIMEOUT   = 5;             % seconds

NUM_LINES      = 200;           % image height (number of line captures)
VALID_PX       = ADC_BUF_LEN;   % pixels to keep PER HALF. Lower it (e.g.
                                % 3600) to crop the dead flat tail seen
                                % after the real sensor line ends.
TRIG_WIDTH     = 2;             % trigger pulse width, CIS CLK cycles
LED_COLOR      = 'w';           % illumination during the scan
LINE_PERIOD    = 0;             % extra pause between lines, seconds
                                % (set to match your motion stage speed)

% If the line alternates video/reset level every other sample (dense
% two-band "black" plot), keep only one phase of the alternation:
DEINTERLACE    = false;         % true -> keep every 2nd sample
PHASE          = 1;             % 1 or 2: which of the two phases to keep

PNG_FILE       = "cis_image.png";
MAT_FILE       = "cis_image_raw.mat";

%% ---- Open port -----------------------------------------------------------
sp = serialport(PORT, 115200, "Timeout", READ_TIMEOUT);  % baud ignored on CDC
flush(sp);
fprintf("Opened %s\n", PORT);

setResolution(sp, TRIG_WIDTH);
setLED(sp, LED_COLOR);
pause(0.05);

%% ---- Capture loop: stack lines into an image -----------------------------
img = [];                       % allocated after the first line (width known)
hFig = figure("Name", "CIS scan"); hIm = [];

for k = 1:NUM_LINES
    [an1, an2] = captureLine(sp, TOTAL_BYTES, BYTES_PER_HALF);
    row = reconstructLine(an1(1:VALID_PX), an2(1:VALID_PX));
    if DEINTERLACE
        row = row(PHASE:2:end);
    end

    if isempty(img)             % first line -> now we know the row width
        img = zeros(NUM_LINES, numel(row));
        hIm = imagesc(img);
        colormap gray; axis image;
        xlabel("pixel"); ylabel("line");
    end
    img(k, :) = row;

    % Live preview every 10 lines (full redraw is slow at 20k px wide)
    if mod(k, 10) == 0 || k == NUM_LINES
        set(hIm, "CData", img);
        title(sprintf("Scanning: line %d / %d", k, NUM_LINES));
        drawnow limitrate;
    end
    if LINE_PERIOD > 0
        pause(LINE_PERIOD);
    end
end

setLED(sp, 'o');
clear sp;                       % closes the port

%% ---- Normalize & save -----------------------------------------------------
% Contrast-stretch between the 1st and 99th percentile so a few outliers
% don't wash out the whole image, then save 8-bit PNG + raw values.
v  = sort(img(:));
lo = v(max(1, round(0.01*numel(v))));
hi = v(round(0.99*numel(v)));
img8 = uint8(255 * min(max((img - lo) / max(hi - lo, 1), 0), 1));

imwrite(img8, PNG_FILE);
save(MAT_FILE, "img");
fprintf("Saved %s (%dx%d) and %s\n", PNG_FILE, size(img8,1), size(img8,2), MAT_FILE);

figure("Name", "Result");
imshow(img8); title("Assembled CIS image");

%% ========================================================================
%% Functions
%% ========================================================================

function setLED(sp, color)
    % color in {'r','g','b','w','o'}
    write(sp, uint8(color), "uint8");
end

function setResolution(sp, triggerWidth)
    % triggerWidth: uint16 trigger pulse width in WHOLE CIS CLK cycles
    % (firmware clamps to TRIG_MAX_CYCLES)
    lo = uint8(bitand(triggerWidth, 255));
    hi = uint8(bitshift(triggerWidth, -8));
    write(sp, uint8('d'), "uint8");
    write(sp, [lo hi], "uint8");
end

function [an1, an2] = captureLine(sp, totalBytes, bytesPerHalf)
    % Send 's', read both halves, return double column vectors.
    flush(sp);
    write(sp, uint8('s'), "uint8");

    raw = read(sp, totalBytes, "uint8");      % read all bytes
    if numel(raw) < totalBytes
        error("captureLine:timeout", ...
              "got %d/%d bytes", numel(raw), totalBytes);
    end
    raw = uint8(raw);

    % Reassemble little-endian uint16 from byte pairs
    half1 = raw(1:bytesPerHalf);
    half2 = raw(bytesPerHalf+1:end);
    an1 = typecast(half1, 'uint16');          % host is little-endian; STM32 too
    an2 = typecast(half2, 'uint16');
    an1 = double(an1(:));
    an2 = double(an2(:));
end

function row = reconstructLine(an1, an2)
    % Combine the two sensor halves into one line.
    % Simplest reconstruction = concatenation (per the paper, Fig. 4).
    % If your sensor interleaves columns instead, replace with:
    %    row = zeros(2*numel(an1),1);
    %    row(1:2:end) = an1;  row(2:2:end) = an2;
    row = [an1; an2];
end
