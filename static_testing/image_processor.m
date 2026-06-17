clc;
clear;
close all;

%% Batch Image Extraction Setup
% Targets standard JPEG files inside the directory
imageFiles = dir('*.jpg'); 

if isempty(imageFiles)
    error('No .jpg target validation images found in this folder.');
end

for iIdx = 1:length(imageFiles)
    currentImgName = imageFiles(iIdx).name;
    fprintf('\n-----------------------------------------\n');
    fprintf('Analyzing Image Grid [%d/%d]: %s\n', iIdx, length(imageFiles), currentImgName);
    fprintf('-----------------------------------------\n');
    
    tic
    originalImage = imread(currentImgName);
    img = imresize(originalImage, 0.5);
    
    % Mask translations to HSV matrices
    hsvImage = rgb2hsv(img);
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
    
    [centers, radii] = imfindcircles(redMask, [50, 400], 'Sensitivity', 0.94, 'EdgeThreshold', 0.1);
    
    grayImage = rgb2gray(img);
    blackMask = grayImage < 30;
    blackMask = bwareaopen(blackMask, 500);
    [B, L] = bwboundaries(blackMask, 'noholes');
    
    squareDetected = false;
    circleWithSquare = [];
    
    figure('Name', ['Target Analysis Plot: ', currentImgName], 'NumberTitle', 'off');
    imshow(img); hold on;
    
    %% Target Coordinate Struct Matching
    for k = 1:length(B)
        stats = regionprops(L == k, 'BoundingBox', 'Extent', 'EulerNumber', 'Image', 'Area');
        if stats.Area < 500, continue; end
        
        aspectRatio = stats.BoundingBox(3) / stats.BoundingBox(4);
        if aspectRatio > 0.9 && aspectRatio < 1.1
            squareDetected = true;
            rectangle('Position', stats.BoundingBox, 'EdgeColor', 'g', 'LineWidth', 2);
            
            squareCenterX = stats.BoundingBox(1) + stats.BoundingBox(3) / 2;
            squareCenterY = stats.BoundingBox(2) + stats.BoundingBox(4) / 2;
            
            for c = 1:length(radii)
                distance = sqrt((centers(c, 1) - squareCenterX)^2 + (centers(c, 2) - squareCenterY)^2);
                if distance <= radii(c)
                    circleWithSquare = c;
                    break;
                end
            end
        end
    end
    
    %% Graphic Drawing Loop
    for c = 1:length(radii)
        if c == circleWithSquare
            viscircles(centers(c, :), radii(c), 'EdgeColor', 'r', 'LineWidth', 2);
        else
            viscircles(centers(c, :), radii(c), 'EdgeColor', 'b', 'LineWidth', 2);
        end
    end
    
    %% Consolidated Engineering Console Prints
    if squareDetected && ~isempty(circleWithSquare)
        x_coord = centers(circleWithSquare, 1);
        y_coord = centers(circleWithSquare, 2);
        image_center_x = size(img, 2) / 2;
        image_center_y = size(img, 1) / 2;
        
        delta_x = x_coord - image_center_x;
        delta_y = y_coord - image_center_y;
        
        fprintf('Target Parameters Validated:\n');
        fprintf('  Computed Radius: %.2f px\n', radii(circleWithSquare));
        fprintf('  Dynamic Shift Vector: Delta X = %.2f px, Delta Y = %.2f px\n', delta_x, delta_y);
        
        text(x_coord + 30, y_coord - 30, ['Radius: ', num2str(radii(circleWithSquare)), ' px'], 'Color', 'yellow', 'FontSize', 12);
        text(x_coord + 30, y_coord - 60, ['\Deltax: ', num2str(delta_x)], 'Color', 'yellow', 'FontSize', 12);
        text(x_coord + 30, y_coord - 90, ['\Deltay: ', num2str(delta_y)], 'Color', 'yellow', 'FontSize', 12);
    else
        disp('Target compound verification failed in this visual matrix.');
    end
    hold off;
    toc
end
