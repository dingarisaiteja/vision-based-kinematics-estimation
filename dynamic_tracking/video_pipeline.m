clc;
clear;
close all;
tic

%% Empirical Constants
ND = 0.46;
VR = 3840;
FPS = 30;
DT = 1 / FPS;

%% Batch Video Processing Setup
% Dynamically scans the folder for all matching AVI files
videoFiles = dir('*.avi'); 

if isempty(videoFiles)
    error('No .avi video files found in the current folder.');
end

% Loop through every single video detected in the workspace directory
for vIdx = 1:length(videoFiles)
    currentVideo = videoFiles(vIdx).name;
    fprintf('\n=========================================\n');
    fprintf('Processing Video [%d/%d]: %s\n', vIdx, length(videoFiles), currentVideo);
    fprintf('=========================================\n');
    
    vidReader = VideoReader(currentVideo);
    
    % Auto-generate the output file name string
    [~, name, ~] = fileparts(currentVideo);
    outputName = ['processed_empirical_', name, '.avi'];
    
    % Initialize VideoWriter with universal cloud-safe container
    outputVideo = VideoWriter(outputName, 'Motion JPEG AVI');
    outputVideo.FrameRate = FPS;
    open(outputVideo);
    
    %% Data Storage Registers
    positions = [];
    velocities = [];
    detected_radii = [];
    
    %% Processing Loop
    while hasFrame(vidReader)
        frame = readFrame(vidReader);
        img = imresize(frame, 0.5);
        hsvImage = rgb2hsv(img);
        
        % Red mask chrominance calculation layers
        lowerRed1 = [0, 0.5, 0.2]; upperRed1 = [0.04, 1, 1];
        lowerRed2 = [0.9, 0.5, 0.2]; upperRed2 = [1, 1, 1];
        
        mask1 = (hsvImage(:,:,1) >= lowerRed1(1) & hsvImage(:,:,1) <= upperRed1(1)) & ...
                (hsvImage(:,:,2) >= lowerRed1(2) & hsvImage(:,:,2) <= upperRed1(2)) & ...
                (hsvImage(:,:,3) >= lowerRed1(3) & hsvImage(:,:,3) <= upperRed1(3));
        mask2 = (hsvImage(:,:,1) >= lowerRed2(1) & hsvImage(:,:,1) <= upperRed2(1)) & ...
                (hsvImage(:,:,2) >= lowerRed2(2) & hsvImage(:,:,2) <= upperRed2(2)) & ...
                (hsvImage(:,:,3) >= lowerRed2(3) & hsvImage(:,:,3) <= upperRed2(3));
                
        redMask = mask1 | mask2;
        redMask = imfill(redMask, 'holes');
        redMask = bwareaopen(redMask, 500);
        
        stats = regionprops(redMask, 'Centroid', 'EquivDiameter');
        
        if ~isempty(stats)
            [~, maxIdx] = max([stats.EquivDiameter]);
            circleCenter = stats(maxIdx).Centroid;
            detected_radius = stats(maxIdx).EquivDiameter / 2;
            detected_radii = [detected_radii; detected_radius];
            
            % Sizing variations relative to image boundaries
            imgCenterX = size(img, 2) / 2;
            imgCenterY = size(img, 1) / 2;
            delta_x = circleCenter(1) - imgCenterX;
            delta_y = circleCenter(2) - imgCenterY;
            
            % Empirical Position Calculations
            Z = (854.02 * (ND / 0.869) * (VR / 4608) * (detected_radius^-1.03));
            X = 0.00253 * ((854.02 * detected_radius^-1.03) / 4.49) * (ND / 0.869) * (4608 / VR) * delta_x;
            Y = -0.0046 * ((854.02 * detected_radius^-1.03) / 7.98) * (ND / 0.869) * (4608 / VR) * delta_y;
            
            positions = [positions; X, Y, Z];
            
            % Discrete Velocities Derivation
            if size(positions, 1) > 1
                Vx = (positions(end,1) - positions(end-1,1)) / DT;
                Vy = (positions(end,2) - positions(end-1,2)) / DT;
                Vz = (positions(end,3) - positions(end-1,3)) / DT;
                V = sqrt(Vx^2 + Vy^2 + Vz^2);
                velocities = [velocities; Vx, Vy, Vz, V];
            end
            
            %% Native Matrix Pixel Marker Insertion (Bypasses Toolbox block entirely)
            % Directly overwrites an 11x11 block around the tracking centroid to red
            cx = round(circleCenter(1)); cy = round(circleCenter(2));
            if cy > 6 && cy < size(img,1)-6 && cx > 6 && cx < size(img,2)-6
                img(cy-5:cy+5, cx-5:cx+5, 1) = 255; % Red Channel Max
                img(cy-5:cy+5, cx-5:cx+5, 2) = 0;   % Green Channel Clear
                img(cy-5:cy+5, cx-5:cx+5, 3) = 0;   % Blue Channel Clear
            end
            
            %% Save Background Frame Stream Directly to Video File
            writeVideo(outputVideo, img);
        end
    end
    
    %% Close video writer asset and dump telemetry metrics to workspace console
    close(outputVideo);
    
    fprintf('\n--- Batch Summary: %s ---\n', currentVideo);
    if ~isempty(velocities)
        fprintf('Avg Vx: %.2f m/s\n', mean(velocities(:,1)));
        fprintf('Avg Vy: %.2f m/s\n', mean(velocities(:,2)));
        fprintf('Avg Vz: %.2f m/s\n', mean(velocities(:,3)));
        fprintf('Avg Net Velocity Speed: %.2f m/s\n', mean(velocities(:,4)));
    else
        disp('Velocity logs unpopulated.');
    end
    if ~isempty(detected_radii)
        fprintf('Avg Extracted Target Pixel Radius: %.2f px\n', mean(detected_radii));
    end
    toc
end
