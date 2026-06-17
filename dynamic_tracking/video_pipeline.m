clc;
clear;
close all;

%% Empirical Constants
ND = 0.46;
VR = 3840;
FPS = 30;
DT = 1 / FPS;

%% Batch Video Processing Setup
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
    [~, name, ~] = fileparts(currentVideo);
    outputName = ['processed_empirical_', name, '.avi'];
    
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
        [h, w, ~] = size(img);
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
            imgCenterX = w / 2;
            imgCenterY = h / 2;
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
            else
                V = 0;
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
            
            %% Directly Draw Over Pixels (Guarantees Perfect Video Output Matrix Size)
            cx = round(circleCenter(1)); 
            cy = round(circleCenter(2));
            
            % Draw a solid 9x9 pixel RED square marker at the center of the circle
            if cy > 5 && cy < h-5 && cx > 5 && cx < w-5
                img(cy-4:cy+4, cx-4:cx+4, 1) = 255; % Red channel max
                img(cy-4:cy+4, cx-4:cx+4, 2:3) = 0; % Clear blue/green channels
            end
            
            % Draw a solid 9x9 pixel GREEN square marker if nested target square is confirmed
            if squareDetected
                scx = round(bestSquare(1) + bestSquare(3)/2);
                scy = round(bestSquare(2) + bestSquare(4)/2);
                if scy > 5 && scy < h-5 && scx > 5 && scx < w-5
                    img(scy-4:scy+4, scx-4:scx+4, 1) = 0;
                    img(scy-4:scy+4, scx-4:scx+4, 2) = 255; % Green channel max
                    img(scy-4:scy+4, scx-4:scx+4, 3) = 0;
                end
            end
            
            % Print real-time values smoothly inside the command terminal instead of window canvas
            if rem(length(detected_radii), 15) == 0
                fprintf('Frame %d Logged -> Coordinates Estimated: X=%.2fm | Y=%.2fm | Z=%.2fm | Speed=%.2fm/s\n', ...
                    length(detected_radii), X, Y, Z, V);
            end
            
            % Write frame directly (unaltered resolution size)
            writeVideo(outputVideo, img);
        end
    end
    
    %% Compute & Display Summary Telemetry Statistics
    close(outputVideo);
    
    fprintf('\n--- Performance Log Summary: %s ---\n', currentVideo);
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
