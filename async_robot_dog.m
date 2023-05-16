function async_robot_dog(Robot_Dog_IP,Robot_Dog_Port,Control_Data)

persistent futureResult;

if isempty(gcp('nocreate'))
    parpool;
end

if isempty(futureResult) || (~strcmp(futureResult.State, 'running') && ~strcmp(futureResult.State, 'queued'))
    futureResult = parfeval(@Robot_Dog, 0, Robot_Dog_IP,Robot_Dog_Port,Control_Data);
end

end

