% multiObjectTracking.m

% This code is 100% credit to the MathWorks organization, the makers of Matlab.  This is a reproducti%on of the code found at:
% 
% http://www.mathworks.com/help/vision/examples/motion-based-multiple-object-tracking.html
% 
% I've only made minor changes to make the MathWorks example to make it compatible with Octave 4.0.0.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function multiObjectTracking()
  
  pkg load image;         % added this line for compatibility with Octave
  
  obj = setupSystemObjects();     % create System objects used for reading video, detecting moving objects, and displaying the results
  
  tracks = initializeTracks();    % create an empty array of tracks
  
  nextId = 1;                     % ID of the next track
  
  while ~isDone(obj.reader)               % detect moving objects, and track them across video frames
    frame = readFrame();
    [centroids, bboxes, mask] = detectObjects(frame);
    predictNewLocationsOfTracks();
    [assignments, unassignedTracks, unassignedDetections] = detectionToTrackAssignment();
    
    updateAssignedTracks();
    updateUnassignedTracks();
    deleteLostTracks();
    createNewTracks();
    
    displayTrackingResults();
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function obj = setupSystemObjects()
    % initialize Video I/O, create objects for reading a video from a file, drawing the tracked objects in each frame, and playing the video
    
    obj.reader = vision.VideoFileReader('768x576.avi');           % create a video file reader
    
    obj.videoPlayer = vision.VideoPlayer('Position', [20, 400, 700, 400]);      % create two video players, one to display the video,
    obj.maskPlayer = vision.VideoPlayer('Position', [740, 400, 700, 400]);      % and one to display the foreground mask
    
    % Create System objects for foreground detection and blob analysis
    
    % The foreground detector is used to segment moving objects from the background. It outputs a binary mask, where the pixel value
    % of 1 corresponds to the foreground and the value of 0 corresponds to the background
    
    obj.detector = vision.ForegroundDetector('NumGaussians', 3, 'NumTrainingFrames', 40, 'MinimumBackgroundRatio', 0.7);
    
    % Connected groups of foreground pixels are likely to correspond to moving objects.  The blob analysis System object is used to find such groups
    % (called 'blobs' or 'connected components'), and compute their characteristics, such as area, centroid, and the bounding box.
    
    obj.blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true, 'AreaOutputPort', true, 'CentroidOutputPort', true, 'MinimumBlobArea', 400);
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function tracks = initializeTracks()
    % create an empty array of tracks
    tracks = struct('id', {}, 'bbox', {}, 'kalmanFilter', {}, 'age', {}, 'totalVisibleCount', {}, 'consecutiveInvisibleCount', {});
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function frame = readFrame()
    frame = obj.reader.step();
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function [centroids, bboxes, mask] = detectObjects(frame)
    
    mask = obj.detector.step(frame);          % detect foreground
    
    mask = imopen(mask, strel('rectangle', [3,3]));             % apply morphological operations to remove noise and fill in holes
    mask = imclose(mask, strel('rectangle', [15, 15]));
    mask = imfill(mask, 'holes');
    
    [~, centroids, bboxes] = obj.blobAnalyser.step(mask);         % perform blob analysis to find connected components
    
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function predictNewLocationsOfTracks()
    
    for i = 1:length(tracks)
      bbox = tracks(i).bbox;
      
      predictedCentroid = predict(tracks(i).kalmanFilter);        % predict the current location of the track
      
      predictedCentroid = int32(predictedCentroid) - bbox(3:4) / 2;     % shift the bounding box so that its center is at the predicted location
      tracks(i).bbox = [predictedCentroid, bbox(3:4)];
      
    end
    
  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function [assignments, unassignedTracks, unassignedDetections] = detectionToTrackAssignment()
    
    nTracks = length(tracks);
    nDetections = size(centroids, 1);
    
    cost = zeros(nTracks, nDetections);                   % compute the cost of assigning each detection to each track
    for i = 1:nTracks
      cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
    end
    
    costOfNonAssignment = 20;               % solve the assignment problem
    [assignments, unassignedTracks, unassignedDetections] = assignDetectionsToTracks(cost, costOfNonAssignment);
    
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function updateAssignedTracks()
    numAssignedTracks = size(assignments, 1);
    
    for i = 1:numAssignedTracks
      trackIdx = assignments(i, 1);
      detectionIdx = assignments(i, 2);
      centroid = centroids(detectionIdx, :);
      bbox = bboxes(detectionIdx, :);
      
      correct(tracks(trackIdx).kalmanFilter, centroid);           % correct the estimate of the object's location using the new detection
      
      tracks(trackIdx).bbox = bbox;         % replace predicted bounding box with detected bounding box
      
      tracks(trackIdx).age = tracks(trackIdx).age + 1;            % update track's age
      
      tracks(trackIdx).totalVisibleCount = tracks(trackIdx).totalVisibleCount + 1;        % update visibility
      tracks(trackIdx).consecutiveInvisibleCount = 0;
    end
    
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function updateUnassignedTracks()
    
    for i = 1:length(unassignedTracks)
      
      ind = unassignedTracks(i);
      tracks(ind).age = tracks(ind).age + 1;
      tracks(ind).consecutiveInvisibleCount = tracks(ind).consecutiveInvisibleCount + 1;
    
    end
  
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function deleteLostTracks()
    if isempty(tracks)
      return;
    end
    
    invisibleForTooLong = 20;
    ageThreshold = 8;
    
    ages = [tracks(:).age];                               % compute the fraction of the track's age for which it was visible
    totalVisibleCounts = [tracks(:).totalVisibleCount];
    visibility = totalVisibleCounts ./ ages;
    
    % find the indices of 'lost' tracks
    lostInds = (ages < ageThreshold & visibility < 0.6) | [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;
    
    tracks = tracks(~lostInds);           % delete lost tracks
    
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function createNewTracks()
    centroids = centroids(unassignedDetections, :);
    bboxes = bboxes(unassignedDetections, :);
    
    for i = 1:size(centroids, 1)
      
      centroid = centroids(i,:);
      bbox = bboxes(i, :);
      
      kalmanFilter = configureKalmanFilter('ConstantVelocity', centroid, [200, 50], [100, 25], 100);      % create a Kalman filter object
      
      % create a new track
      newTrack = struct('id', nextId, 'bbox', bbox, 'kalmanFilter', kalmanFilter, 'age', 1, 'totalVisibleCount', 1, 'consecutiveInvisibleCount', 0);
      
      tracks(end + 1) = newTrack;             % add it to the array of tracks
      
      nextId = nextId + 1;                    % increment the next id
      
    end
    
  end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  function displayTrackingResults()
    
    frame = im2uint8(frame);                          % convert the frame and the mask to uint8 RGB
    mask = uint8(repmat(mask, [1, 1, 3])) .* 255;
    
    minVisibleCount = 8;
    
    if ~isempty(tracks)
      
      % noisy detections tend to result in short-lived tracks, only display tracks that have been visible for more than a minimum number of frames
      reliableTrackInds = [tracks(:).totalVisibleCount] > minVisibleCount;
      
      reliableTracks = tracks(reliableTrackInds);
      
      % display the objects, if an object has not been detected in this frame, display its predicted bounding box
      if ~isempty(reliableTracks)
        
        bboxes = cat(1, reliableTracks.bbox);             % get bounding boxes
        
        ids = int32([reliableTracks(:).id]);              % get ids
        
        % Create labels for objects indicating the ones for
        % which we display the predicted rather than the actual
        % location.
        labels = cellstr(int2str(ids'));
        predictedTrackInds = [reliableTracks(:).consecutiveInvisibleCount] > 0;
        isPredicted = cell(size(labels));
        isPredicted(predictedTrackInds) = {' predicted'};
        labels = strcat(labels, isPredicted);
        
        % Draw the objects on the frame.
        frame = insertObjectAnnotation(frame, 'rectangle', bboxes, labels);
        
        % Draw the objects on the mask.
        mask = insertObjectAnnotation(mask, 'rectangle', bboxes, labels);
        
      end     % end inner if
      
    end     % end outer if
    
    % Display the mask and the frame.
    obj.maskPlayer.step(mask);
    obj.videoPlayer.step(frame);
  
  end   % end function

end   % end outer function
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    