function error = objFunc(dh_parameters)
% The purpose of this is to get the error between our measured (actual
% values) and our predicted values generated from fkine by using the
% euclidean distance.

% data is going to be 5 sets of our joint angle positions that will be used
% to optimize the dh parameters
data = [pi/2 0 0 7.25; 
    pi/2 pi/2 0 5.25; 
    0 -pi/2 0 5.25;
    0 pi/6 0 7.5;
    pi/6 pi/6 0 7.5];

% Based off the new dh parameters we pass them as an argument to our link
% parameter, take note of the () position corresponds to matrix above
L(1) = Link('a',dh_parameters(1) ,'alpha', dh_parameters(2), 'd', dh_parameters(3), 'offset', dh_parameters(4));
L(2) = Link('a', dh_parameters(5), 'alpha', dh_parameters(6) , 'd', dh_parameters(7), 'offset', dh_parameters(8));
L(3) = Link('a', dh_parameters(9) , 'alpha', dh_parameters(10), 'd', dh_parameters(11), 'offset', dh_parameters(12));
handles.robot.base = [1 0 0 0; 0 1 0 8; 0 0 1 0; 0 0 0 1];
handles.robot = SerialLink(L, 'name', 'robot');

%initalize error to be 0 it is an accumulator variable. 
error = 0; 
for i = 1:length(data)
    % fkine will give our predicted transformation matrix that holds our  
    %x and y position of the end effector
    predict(i) = handles.robot.fkine([data(i,1), data(i,2), data(i,3)]);
    % using the pythagorem theorm to get the distance based off x and y
    predict_pythagreom(i) = sqrt((predict(1,i).t(1))^2 + (predict(1,i).t(2))^2);
    % this code will sum all the error with each iteration note the
    % indexing utilized from the for loop. 
    error = error + sqrt((data(i,4) - predict_pythagreom(i))^2);
    
end
% this will give us the average error 
error = error/5

end

