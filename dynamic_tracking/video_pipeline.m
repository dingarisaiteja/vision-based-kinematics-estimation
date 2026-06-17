clc;
clear;
close all;

%% Empirical Constants
ND = 0.46;
VR = 3840;
FPS = 30;
DT = 1 / FPS;

%% Batch Video Processing Setup
% Scans current directory for all AVI videos
videoFiles = dir('*.avi'); 

if isempty(videoFiles)
    error('No .avi video files found in the current folder.');
end

% Loop through every video detected
for vIdx = 1:length(videoFiles)
    currentVideo = videoFiles(vIdx).name;
    fprintf('\n=========================================\n');
    fprintf('Processing Video [%d/%d]: %s\n', vIdx, length(videoFiles), currentVideo);
    fprintf('=========================================\n');
    
    tic
    vidReader = VideoReader(currentVideo);
    
    % Generate output string name automatically
    [~, name, ext] = fileparts(currentVideo);
    outputName = ['processed_empirical_', name, ext];
    
    outputVideo = VideoWriter(outputName, 'MPEG-4');
    outputVideo.FrameRate = FPS;
    open(outputVideo);
    
    % Initialize Live Visualizer Window Frame
    hFig = figure('Name', ['Tracking Live Feed: ', currentVideo], 'NumberTitle', 'off');
    
    %% Data Storage Registers
    positions = [];
    velocities = [];
    detected_radii = [];
    
    %% Processing Loop
    while hasFrame(vidReader) && ishandle(hFig)
        frame = readFrame(vidReader);
        img = imresize(frame, 0.5);
        hsvImage = rgb2hsv(img);
        
        % Red mask chrominance bounds
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
            
            % Image spatial sizing arrays
            imgCenterX = size(img, 2) / 2;
            imgCenterY = size(img, 1) / 2;
            delta_x = circleCenter(1) - imgCenterX;
            delta_y = circleCenter(2) - imgCenterY;
            
            % Empirical Kinematic Position Calculations
            Z = (854.02 * (ND / 0.869) * (VR / 4608) * (detected_radius^-1.03));
            X = 0.00253 * ((854.02 * detected_radius^-1.03) / 4.49) * (ND / 0.869) * (4608 / VR) * delta_x;
            Y = -0.0046 * ((854.02 * detected_radius^-1.03) / 7.98) * (ND / 0.869) * (4608 / VR) * delta_y;
            
            positions = [positions; X, Y, Z];
            
            % Discrete Temporal Derivative Integration
            if size(positions, 1) > 1
                Vx = (positions(end,1) - positions(end-1,1)) / DT;
                Vy = (positions(end,2) - positions(end-1,2)) / DT;
                Vz = (positions(end,3) - positions(end-1,3)) / DT;
                V = sqrt(Vx^2 + Vy^2 + Vz^2);
                velocities = [velocities; Vx, Vy, Vz, V];
            end
            
            %% Square Matrix Tracking Verification
            grayImage = rgb2gray(img);
            blackMask = grayImage < 40;
            blackMask = bwareaopen(blackMask, 500);
            statsSquares = regionprops(blackMask, 'BoundingBox', 'Centroid');
            squareDetected = false;
            
            for k = 1:length(statsSquares)
                ar = statsSquares(k).BoundingBox(3) / statsSquares(k).BoundingBox(4);
                if ar > 0.9 && ar < 1.1
                    squareCenter = statsSquares(k).Centroid;
                    if norm(circleCenter - squareCenter) <= detected_radius
                        bestSquare = statsSquares(k).BoundingBox;
                        squareDetected = true;
                        break;
                    end
                end
            end
            
            %% Multi-Channel Structural Annotations
            img = insertShape(img, 'Circle', [circleCenter detected_radius], 'Color', 'red', 'LineWidth', 3);
            img = insertText(img, [30 50],  sprintf('Z: %.2f m', Z), 'FontSize', 35, 'TextColor', 'yellow', 'BoxColor', 'black');
            img = insertText(img, [30 100], sprintf('X: %.2f m', X), 'FontSize', 35, 'TextColor', 'yellow', 'BoxColor', 'black');
            img = insertText(img, [30 150], sprintf('Y: %.2f m', Y), 'FontSize', 35, 'TextColor', 'yellow', 'BoxColor', 'black');
            img = insertText(img, [30 200], sprintf('Radius: %.1f px', detected_radius), 'FontSize', 35, 'TextColor', 'green', 'BoxColor', 'black');
            
            if ~isempty(velocities)
                img = insertText(img, [30 250], sprintf('V: %.2f m/s', V), 'FontSize', 35, 'TextColor', 'cyan', 'BoxColor', 'black');
            end
            if squareDetected
                img = insertShape(img, 'Rectangle', bestSquare, 'Color', 'green', 'LineWidth', 3);
            end
            
            %% Visual Display Feedback & Storage Routines
            imshow(img); % Displays current calculated frame on display screen
            drawnow;     % Forces MATLAB to refresh display instantly
            
            writeVideo(outputVideo, img);
        end
    end
    
    %% Compute & Display Summary Telemetry Statistics
    close(outputVideo);
    if ishandle(hFig), close(hFig); end
    
    fprintf('\n--- Performance Log: %s ---\n', currentVideo);
    if ~isempty(velocities)
        fprintf('Avg Vx: %.2f m/s | Avg Vy: %.2f m/s | Avg Vz: %.2f m/s\n', mean(velocities(:,1)), mean(velocities(:,2)), mean(velocities(:,3)));
        fprintf('Compiled Mean Translation Velocity Speed: %.2f m/s\n', mean(velocities(:,4)));
    else
        disp('Velocity matrices unpopulated.');
    end
    if ~isempty(detected_radii)
        fprintf('Mean Captured Target Matrix Dimension: %.2f px\n', mean(detected_radii));
    end
    toc
end
