clc;close all;clear all;

% Camera initialization and setup
webcamList = webcamlist;
disp('Connected cameras:');
disp(webcamList);
% Identify indices for the 'HD USB Camera'
hdUsbCameraIndices = find(strcmp(webcamList, 'HD USB Camera'));
% Check if there are at least two 'HD USB Camera'
if length(hdUsbCameraIndices) < 2
    error('Not enough HD USB Cameras found.');
end

% Create webcam objects
videoReader_1 = webcam(hdUsbCameraIndices(1));
videoReader_1.Resolution = '1920x1080';
videoReader_2 = webcam(hdUsbCameraIndices(2));
videoReader_2.Resolution = '1920x1080';
videoPlayer_1 = vision.VideoPlayer('Position',[100,100,700,525]);%set visualization window size// 'Position',[100,100,680,520]
videoPlayer_2 = vision.VideoPlayer('Position',[100,100,700,525]);
load('ans.mat');


%% video recording
opengl('software') ;
global writerObj
writerObj= VideoWriter('sheath_150V3','Motion JPEG AVI');
open(writerObj);

%% Main
while (1)
    frame_1 = snapshot(videoReader_1);
    frame_2 = snapshot(videoReader_2);
    [sheath_tip] = visualCom(frame_1,frame_2,ans);
end


function [tip_1,tip_2,sheath_tip] = sheath_seg(frame_1,frame_2,stereoParams)
    
    frame_1hsv = rgb2hsv(frame_1);
    frame_2hsv = rgb2hsv(frame_2);
    
    lower_green = [0.2,0.3,0.3];
    upper_green = [0.5,1,1];
    h_min = lower_green(1);s_min = lower_green(2);v_min = lower_green(3);
    h_max = upper_green(1);s_max = upper_green(2);v_max = upper_green(3);
    seg_1 = (frame_1hsv(:,:,1) >= h_min ) & (frame_1hsv(:,:,1) <= h_max) &...
            (frame_1hsv(:,:,2) >= s_min ) & (frame_1hsv(:,:,2) <= s_max) &...
            (frame_1hsv(:,:,2) >= v_min ) & (frame_1hsv(:,:,2) <= v_max);
    seg_2 = (frame_2hsv(:,:,1) >= h_min ) & (frame_2hsv(:,:,1) <= h_max) &...
            (frame_2hsv(:,:,2) >= s_min ) & (frame_2hsv(:,:,2) <= s_max) &...
            (frame_2hsv(:,:,2) >= v_min ) & (frame_2hsv(:,:,2) <= v_max);
    
    %backbone extraction
    se = strel('line',25,90);
    erodedBW_1 = imerode(seg_1,se);
    erodedBW_2 = imerode(seg_2,se);

    %tip extraction
    [x1,y1] = find(erodedBW_1);
    [x2,y2] = find(erodedBW_2);
    [ y1_min, y1_min_ind ] = min(y1);
    tip_1 = [y1_min x1(y1_min_ind)];
    [ y2_min, y2_min_ind ] = min(y2);
    tip_2 = [y2_min x2(y2_min_ind)];

    if (all(tip_1 == 0)|all(tip_2 == 0))
        sheath_tip = [0,0,0];
    else
        sheath_tip = triangulate(tip_1,tip_2,stereoParams);% pixel coordinates to vision coordinates
    end
    disp(sheath_tip);
end


function [sheath_tip] = visualCom(frame_1,frame_2,stereoParams)
    % global writerObj;
    [tip_1,tip_2,sheath_tip] = sheath_seg(frame_1,frame_2,stereoParams);

    overlay_1 = insertShape(frame_1,'Circle',[tip_1(1) tip_1(2) 10],'LineWidth',15);
    overlay_2 = insertShape(frame_2,'Circle',[tip_2(1) tip_2(2) 10],'LineWidth',15);

    subplot(1,2,1),imshow(overlay_1);title('Left View')
    subplot(1,2,2),imshow(overlay_2);title('Right View')
    % set(gcf,'renderer','OpenGL');
    % thisFrame = getframe(gcf);    
    % writeVideo(writerObj, thisFrame);
    disp(sheath_tip);
end




        