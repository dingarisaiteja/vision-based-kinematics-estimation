clc;
clear;
close all;
tic

%% Empirical Constants
ND = 0.46;
VR = 3840;
FPS = 30;
DT = 1 / FPS;

%% Video Setup
videoFile = "Z14.avi";
vidReader = VideoReader(videoFile);

% Create output video writer
outputVideo = VideoWriter('processed_empirical_Z14.avi', 'MPEG-4');
outputVideo.FrameRate = FPS;
open(outputVideo);

%% Data Storage
positions = [];
velocities = [];
detected_radii = [];

%% Processing Loop
while hasFrame(vidReader)
    frame = readFrame(vidReader);
    img = imresize(frame, 0.5);
    hsvImage = rgb2hsv(img);

    % Red mask (two ranges)
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

        % Image center
        imgCenterX = size(img, 2) / 2;
        imgCenterY = size(img, 1) / 2;
        delta_x = circleCenter(1) - imgCenterX;
        delta_y = circleCenter(2) - imgCenterY;

        % Empirical Model (Z, X, Y)
        Z =   (854.02 * (ND / 0.869) * (VR / 4608) * (detected_radius^-1.03));
        X =   0.00253 * ((854.02 * detected_radius^-1.03) / 4.49) * (ND / 0.869) * (4608 / VR) * delta_x;
        Y =  -0.0046 * ((854.02 * detected_radius^-1.03) / 7.98) * (ND / 0.869) * (4608 / VR) * delta_y;

        positions = [positions; X, Y, Z];

        % Velocity Calculation
        if size(positions, 1) > 1
            Vx = (positions(end,1) - positions(end-1,1)) / DT;
            Vy = (positions(end,2) - positions(end-1,2)) / DT;
            Vz = (positions(end,3) - positions(end-1,3)) / DT;
            V = sqrt(Vx^2 + Vy^2 + Vz^2);
            velocities = [velocities; Vx, Vy, Vz, V];
        end

        %% Square Detection (Inside Circle)
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

        %% Annotate Frame
        img = insertShape(img, 'Circle', [circleCenter detected_radius], 'Color', 'red', 'LineWidth', 3);
        img = insertText(img,  [30 50], sprintf('Z: %.2f m', Z), 'FontSize', 35, 'TextColor', 'yellow', 'BoxColor', 'black');
        img = insertText(img,  [30 100], sprintf('X: %.2f m', X), 'FontSize', 35, 'TextColor', 'yellow', 'BoxColor', 'black');
        img = insertText(img,  [30 150], sprintf('Y: %.2f m', Y), 'FontSize', 35, 'TextColor', 'yellow', 'BoxColor', 'black');
        img = insertText(img,  [30 200], sprintf('Radius: %.1f px', detected_radius), 'FontSize', 35, 'TextColor', 'green', 'BoxColor', 'black');

        if ~isempty(velocities)
            img = insertText(img, [30 250], sprintf('V: %.2f m/s', V), 'FontSize', 35, 'TextColor', 'cyan', 'BoxColor', 'black');
        end

        if squareDetected
            img = insertShape(img, 'Rectangle', bestSquare, 'Color', 'green', 'LineWidth', 3);
        end

        %% Save to Video
        writeVideo(outputVideo, img);
    end
end

%% Final Stats
close(outputVideo);
fprintf('\n--- Final Results ---\n');
if ~isempty(velocities)
    fprintf('Avg Vx: %.2f m/s\n', mean(velocities(:,1)));
    fprintf('Avg Vy: %.2f m/s\n', mean(velocities(:,2)));
    fprintf('Avg Vz: %.2f m/s\n', mean(velocities(:,3)));
    fprintf('Avg Speed: %.2f m/s\n', mean(velocities(:,4)));
else
    disp('No velocities calculated.');
end

if ~isempty(detected_radii)
    fprintf('Avg Radius: %.2f px\n', mean(detected_radii));
else
    disp('No circles detected.');
end
toc
