clc;
clear;
close all;
tic

% Define the input image
imagePath = "s005.jpg"; % Change to your image path
originalImage = imread(imagePath);
img = imresize(originalImage, 0.5);

% Display Original Image
figure; imshow(img); title('Original Image');

% Convert to HSV for red color detection
hsvImage = rgb2hsv(img);

% Define red color mask
lowerRed1 = [0, 0.5, 0.2];
upperRed1 = [0.04, 1, 1];
lowerRed2 = [0.9, 0.5, 0.2];
upperRed2 = [1, 1, 1];

mask1 = (hsvImage(:,:,1) >= lowerRed1(1) & hsvImage(:,:,1) <= upperRed1(1)) & ...
        (hsvImage(:,:,2) >= lowerRed1(2) & hsvImage(:,:,2) <= upperRed1(2)) & ...
        (hsvImage(:,:,3) >= lowerRed1(3) & hsvImage(:,:,3) <= upperRed1(3));

mask2 = (hsvImage(:,:,1) >= lowerRed2(1) & hsvImage(:,:,1) <= upperRed2(1)) & ...
        (hsvImage(:,:,2) >= lowerRed2(2) & hsvImage(:,:,2) <= upperRed2(2)) & ...
        (hsvImage(:,:,3) >= lowerRed2(3) & hsvImage(:,:,3) <= upperRed2(3));

redMask = mask1 | mask2;

% Display Red Mask
figure; imshow(redMask); title('Red Mask');

% Morphological processing
redMask = imfill(redMask, 'holes');
redMask = bwareaopen(redMask, 500);

% Display Processed Red Mask
figure; imshow(redMask); title('Red Mask After Morphological Processing');

% Detect circles in the red mask
[centers, radii] = imfindcircles(redMask, [50, 400], 'Sensitivity', 0.94, 'EdgeThreshold', 0.1);

% Convert image to grayscale for black square detection
grayImage = rgb2gray(img);

% Display Grayscale Image
figure; imshow(grayImage); title('Grayscale Image for Black Square Detection');

% Define black color mask
blackMask = grayImage < 30;
blackMask = bwareaopen(blackMask, 500);

% Display Black Mask
figure; imshow(blackMask); title('Black Mask for Square Detection');

% Detect boundaries of objects in the black mask
[B, L] = bwboundaries(blackMask, 'noholes');

% Initialize variables
squareDetected = false;
circleWithSquare = [];

% Final Detection Result - Display Image
figure; imshow(img); hold on;
title('Final Detection Result');

% Loop through each detected boundary for square detection
for k = 1:length(B)
    stats = regionprops(L == k, 'BoundingBox', 'Extent', 'EulerNumber', 'Image', 'Area');

    if stats.Area < 500
        continue;
    end

    aspectRatio = stats.BoundingBox(3) / stats.BoundingBox(4);
    if aspectRatio > 0.9 && aspectRatio < 1.1
        squareDetected = true;
        rectangle('Position', stats.BoundingBox, 'EdgeColor', 'g', 'LineWidth', 2); % Green box around square

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

% Draw Circles and Annotate
for c = 1:length(radii)
    if c == circleWithSquare
        viscircles(centers(c, :), radii(c), 'EdgeColor', 'r', 'LineWidth', 2); % Red for circle with square
        text(centers(c,1) + 30, centers(c,2) - 30, ['Radius: ', num2str(radii(c)), ' px'], 'Color', 'yellow', 'FontSize', 12, 'HorizontalAlignment', 'left');
    else
        viscircles(centers(c, :), radii(c), 'EdgeColor', 'b', 'LineWidth', 2); % Blue for other circles
    end
end

% Display detection result
if squareDetected && ~isempty(circleWithSquare)
    disp(['Target Detected in Red Circle:']);
    disp(['  Radius (pixels) = ', num2str(radii(circleWithSquare))]);
    disp(['  X Coordinate = ', num2str(centers(circleWithSquare, 1))]);
    disp(['  Y Coordinate = ', num2str(centers(circleWithSquare, 2))]);

    % Calculate delta_x and delta_y for the detected circle
    x_coord = centers(circleWithSquare, 1);
    y_coord = centers(circleWithSquare, 2);

    % Calculate the image center
    image_center_x = size(img, 2) / 2;
    image_center_y = size(img, 1) / 2;

    % Calculate deltas
    delta_x = x_coord - image_center_x;
    delta_y = y_coord - image_center_y;

    % Display delta values
    disp(['  Δx = ', num2str(delta_x)]);
    disp(['  Δy = ', num2str(delta_y)]);

    % Annotate the image with delta values and radius
    text(x_coord + 30, y_coord - 30, ['Radius: ', num2str(radii(circleWithSquare)), ' px'], 'Color', 'yellow', 'FontSize', 12, 'HorizontalAlignment', 'left');
    text(x_coord + 30, y_coord - 60, ['Δx: ', num2str(delta_x)], 'Color', 'yellow', 'FontSize', 12, 'HorizontalAlignment', 'left');
    text(x_coord + 30, y_coord - 90, ['Δy: ', num2str(delta_y)], 'Color', 'yellow', 'FontSize', 12, 'HorizontalAlignment', 'left');
else
    disp('No circle containing a black square was detected.');
end

hold off;
toc
