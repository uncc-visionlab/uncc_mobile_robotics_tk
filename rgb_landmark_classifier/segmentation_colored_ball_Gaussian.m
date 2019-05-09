clear;
clc;
close all;
%video_filename = 'data/test_video.mp4';
%in_video_filename = 'out-1.ogv';
in_video_filename = 'out-1-cropped.avi';
out_video_filename = 'tracked_balls.avi';
INTERACTIVE_TRAINING = true;
TRAIN_FROM_IMAGE = true
OUTPUT_SIDE_BY_SIDE_VIDEO = true;
USE_WEBCAM = false;
numClasses = 5;

%% TRAIN THE CLASSIFIERS
if (INTERACTIVE_TRAINING == true)
    if (~TRAIN_FROM_IMAGE)
        video = VideoReader(in_video_filename);
        numImages = video.Duration*video.FrameRate;
        trainFrameIdx = round(numImages/3);
        index=1;
        while video.hasFrame()
            videoFrame = video.readFrame();
            if (index == trainFrameIdx)
                trainFrame = videoFrame;
                figure(1), imshow(trainFrame);
            end
            index = index + 1;
        end
    else
        trainFrame = imread('SLAM_landmark_training_image.png');
        figure(1), imshow(trainFrame);
    end
    strVal = sprintf('You will need to select %d different ellipse-shaped object regions.', numClasses);
    message = {strVal ...
        'Each region should select a specific class for detection' ...
        'Press ESC after each region is specified to move to the next region'};
   
    waitfor(msgbox(message, 'Select Training Data','help'))
    classInfo = cell(numClasses);
   
    [rows,cols,comps] = size(trainFrame);
    pixelVec = reshape(trainFrame, rows*cols, comps);
    for classIdx=1:numClasses
        ellipseHandle = imellipse();
        wait(ellipseHandle);
        classInfo{classIdx}.index = classIdx;
        %classInfo{classIdx}.trainingEllipse = ellipseHandle;
        classInfo{classIdx}.name = sprintf('%d',classIdx);
        mask = ellipseHandle.createMask();
        ind = find(mask > 0);
        classInfo{classIdx}.RGBValues = double(pixelVec(ind,:));
        classInfo{classIdx}.Gaussian.mean = mean(classInfo{classIdx}.RGBValues);
        classInfo{classIdx}.Gaussian.cov = cov(classInfo{classIdx}.RGBValues);       
        classInfo{classIdx}.color = classInfo{classIdx}.Gaussian.mean./(2^8-1);
    end   
    save('class_training_data.mat','classInfo');
else
    load('class_training_data.mat');
end
return

%% APPLY THE CLASSIFIERS
% Blurring window size
winSize=7;
% Detection Negative Log Probability threshold for pixel classified as part of object
% detected_pixel_class_c = -log(p(x|class=c)) < NLOG_PROB_THRESHOLD)
NLOG_PROB_THRESHOLD = 13;
% Detection Region size threshold for pixel region classified as object
% detected_object_class_c = region.area_pixels > MIN_BALL_SIZE_PIXELS
MIN_BALL_SIZE_PIXELS = 25;
% Maximum allowable error between a track and a detection
% Used to assign detected object to existing object tracks
UNASSIGNED_COST = 99;
%STATE_EVOLUTION_MODEL = 'ConstantVelocity';
STATE_EVOLUTION_MODEL = 'ConstantAcceleration';

if (USE_WEBCAM)
    livecam = webcam(1);
    cols_rows = sscanf(livecam.Resolution,'%dx%d');
    rows = cols_rows(2);
    cols = cols_rows(1);
else
    video = VideoReader(in_video_filename);
    rows = video.Height;
    cols = video.Width;
end
comps = 3;

tracks=[];
numTracks=0;
numActiveTracks=0;

gaussFilter = zeros(winSize,winSize);
for x=1:winSize
    for y=1:winSize
        gaussFilter(y,x)=mvnpdf([x;y],0.5*(winSize+1)*ones(2,1), ...
            (0.3*winSize)^2*eye(2));
    end
end
gaussFilter = gaussFilter./sum(sum(gaussFilter));

classLogProb = zeros(rows*cols, numClasses);

frameIndex=1;
while video.hasFrame()
%while (frameIndex < 100)
    if (USE_WEBCAM)
        videoFrame = double(livecam.snapshot());
    else
        videoFrame = double(video.readFrame());
    end
    blurFrame = imfilter(videoFrame, gaussFilter, 'symmetric');
    pixelVec = reshape(blurFrame, rows*cols, comps);
    if (frameIndex < 3)
        frameIndex = frameIndex + 1;
        continue;
    elseif (frameIndex == 3) % used for background subtraction
        firstFrameVec = pixelVec;
    end
   
    % classify pixels using Gaussian distribution model
    for classIdx=1:numClasses
        classLogProb(:,classIdx) = -log(mvnpdf(pixelVec, ...
            classInfo{classIdx}.Gaussian.mean, ...
            classInfo{classIdx}.Gaussian.cov));
    end
    [values, classIdxs] = min(classLogProb');
    classIdxs(values > NLOG_PROB_THRESHOLD) = 10;

    % use background subtraction to classify background pixels
    %diffFrameVec = sum(abs(firstFrameVec - pixelVec)')';
    %classIdxs(diffFrameVec < 30) = 10;
   
    segmentation = reshape(classIdxs,rows,cols);

    % show segmentation result
    Lrgb = label2rgb(segmentation, 'jet', 'w', 'shuffle');
    figure(1), subplot(1,2,1), imshow(uint8(videoFrame));
    figure(1), subplot(1,2,2), imshow(Lrgb), ....
        title('Color segmentation');
    drawnow;
    frameIndex
    frameIndex = frameIndex + 1;    
end