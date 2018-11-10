
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [z] = DoObservationModel(xVehicle, xFeature)
    % get the distance to the landmarks
    DeltaObs = xFeature-xVehicle(1:2);
    z = [norm(DeltaObs)];
end