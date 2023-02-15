classdef LidarVehicleDetector
    % ----------------------------------------------------------------------
    % Step 1: Define the required properties describing the algorithm. This
    % includes Name, Description, and UserDirections.
    properties(Constant)
        
        % Name Algorithm Name
        %   Character vector specifying the name of the algorithm.
        Name = 'Lidar Vehicle Detector';
        
        % Description Algorithm Description
        %   Character vector specifying the short description of the algorithm.
        Description = 'Detect vehicles in point cloud using the pretrained PointPillars object detector.';
        
        % UserDirections Algorithm Usage Directions
        %   Cell array of character vectors specifying directions for
        %   algorithm users to follow to use the algorithm.
        UserDirections = {['ROI Label Definition Selection: select one of ' ...
            'the ROI definitions to be labeled'], ...
            ['Run: Press RUN to run the automation algorithm. '], ...
            ['Review and Modify: Review automated labels over the interval ', ...
            'using playback controls. Modify/delete/add ROIs that were not ' ...
            'satisfactorily automated at this stage. If the results are ' ...
            'satisfactory, click Accept to accept the automated labels.'], ...
            ['Change Settings and Rerun: If automated results are not ' ...
            'satisfactory, you can try to re-run the algorithm with ' ...
            'different settings. To do so, click Undo Run to undo ' ...
            'current automation run, click Settings, make changes to Settings,' ...
            'and press Run again.'], ...
            ['Accept/Cancel: If the results of automation are satisfactory, ' ...
            'click Accept to accept all automated labels and return to ' ...
            'manual labeling. If the results of automation are not ' ...
            'satisfactory, click Cancel to return to manual labeling ' ...
            'without saving the automated labels.']};
    end

    % ---------------------------------------------------------------------
    % Step 2: Define properties you want to use during the algorithm
    % execution.
    properties
        
        % SelectedLabelName 
        %   Name of the selected label. Vehicles detected by the algorithm 
        %   are assigned this variable name.
        SelectedLabelName
        
        % PretrainedDetector
        %   PretrainedDetector saves the pretrained PointPillars object 
        %   detector.
        PretrainedDetector
        
        % ConfidenceThreshold
        %  Specify the confidence threshold to use only detections with 
        %  confidence scores above this value.
        ConfidenceThreshold = 0.45;           
        
    end
    methods

       function isValid = checkSignalType(signalType)            
               % Only point cloud signal data is valid for the Lidar Vehicle
               % detector algorithm.
               isValid = (signalType == vision.labeler.loading.SignalType.PointCloud);           
       end

       function isValid = checkLabelDefinition(~,labelDef)            
            % Only cuboid ROI label definitions are valid for the Lidar
            % vehicle detector algorithm.
            isValid = labelDef.Type == labelType.Cuboid;
       end

       function isReady = checkSetup(algObj)            
            % Is there one selected ROI Label definition to automate.
            isReady = ~isempty(algObj.SelectedLabelDefinitions);
       end

       function settingsDialog(algObj)
            % Invoke dialog with option for modifying the confidence threshold. 
            lidarVehicleDetectorSettings(algObj)
       end

       function initialize(algObj,~)           
            % Store the name of the selected label definition. Use this
            % name to label the detected vehicles.
            algObj.SelectedLabelName = algObj.SelectedLabelDefinitions.Name;
            
            % Load the pretrained pointPillarsObjectDetector.
            pretrainedDetector = load('pretrainedPointPillarsDetector.mat','detector');
            algObj.PretrainedDetector = pretrainedDetector.detector;          
       end

       function autoLabels = run(algObj,ptCloud)           
            bBoxes = [];
            for i = 1:2
                if i == 2
                    % Rotate the point cloud by 180 degrees.
                    theta = 180;
                    trans = [0, 0, 0];
                    tform = rigidtform3d([0 0 theta], trans);
                    ptCloud = pctransform(ptCloud,tform);
                end

                % Detect the bounding boxes using the pretrained detector.
                [box,~,~] = detect(algObj.PretrainedDetector,ptCloud, ...
                    "Threshold",algObj.ConfidenceThreshold);

                if ~isempty(box)
                    if i == 2
                        box(:,1) = -box(:,1);
                        box(:,2) = -box(:,2);
                        box(:,9) = -box(:,9);
                    end
                    bBoxes = [bBoxes;box];
                end
            end

            if ~isempty(bBoxes)
                % Add automated labels at bounding box locations detected
                % by the vehicle detector, of type Cuboid and with the name
                % of the selected label.
                autoLabels.Name     = algObj.SelectedLabelName;
                autoLabels.Type     = labelType.Cuboid;
                autoLabels.Position = bBoxes;
            else
                autoLabels = [];
            end
       end
       



   end
    



end 