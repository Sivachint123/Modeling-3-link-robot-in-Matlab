function varargout = Project_1(varargin)
% PROJECT_1 MATLAB code for Project_1.fig
%      PROJECT_1, by itself, creates a new PROJECT_1 or raises the existing
%      singleton*.
%
%      H = PROJECT_1 returns the handle to a new PROJECT_1 or the handle to
%      the existing singleton*.
%
%      PROJECT_1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROJECT_1.M with the given input arguments.
%
%      PROJECT_1('Property','Value',...) creates a new PROJECT_1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Project_1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Project_1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Project_1

% Last Modified by GUIDE v2.5 12-Oct-2019 18:02:53

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Project_1_OpeningFcn, ...
                   'gui_OutputFcn',  @Project_1_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Project_1 is made visible.
function Project_1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Project_1 (see VARARGIN)

% Choose default command line output for Project_1
handles.output = hObject;

% the robot initalized to 0 
handles.robot = [];

% Robots joint variables 
handles.theta = [0, 0, 0];

% This code is needed to make sure our variables retain there values
% in other functions 
guidata(hObject, handles);

% UIWAIT makes Project_1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Project_1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function Joint_1_Callback(hObject, eventdata, handles)

% This code will grab the value of our slider position and store it to our
% structs variable theta(1)
handles.theta(1) = get(hObject,'Value');

% with the new theta value we will plot our robot with the new angle 
handles.robot.plot (handles.theta);

% The code below will be used to update end effector x position in 
% real time as we move the slider
T = handles.robot.fkine(handles.theta);
X = sprintf('X = %f', T.t(1));
set(handles.x_func,'String',X);
drawnow();

% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function Joint_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function Joint_2_Callback(hObject, eventdata, handles)

% This code will grab the value of our slider position and store it to our
% structs variable theta(2)
handles.theta(2) = get(hObject,'Value');

% with the new theta value we will plot our robot with the new angle 
handles.robot.plot(handles.theta);

% The code below will be used to update end effector x position in 
% real time as we move the slider
T = handles.robot.fkine(handles.theta);
Y = sprintf('Y = %f', T.t(2));
set(handles.y_func,'String',Y);
drawnow();

% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function Joint_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Joint_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
% --- Executes on slider movement.
function End_Effector_Callback(hObject, eventdata, handles)

% This code will grab the value of our slider position and store it to our
% structs variable theta(3)
handles.theta(3) = get(hObject,'Value');

%with the new theta value we will plot our robot with the new angle 
handles.robot.plot(handles.theta);

% The code below will be used to update end effector x position in 
% real time as we move the slider
T = handles.robot.fkine(handles.theta);
Z = sprintf('Z = %f', T.t(3));
set(handles.z_func,'String',Z);
drawnow();
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function End_Effector_CreateFcn(hObject, eventdata, handles)
% hObject    handle to End_Effector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



% --- Executes on button press in Create_Robot.
function Create_Robot_Callback(hObject, eventdata, handles)

% value of our work space to change the floor titles
W = [-10 15 0 20 -5 5];

% deg will be used to convert degrees to radians 
deg = pi/180;
% By inputting our dh parameters as an argument we will generate each link
L(1) = Link('a', 3.65,'alpha', 0, 'd', 1.75, 'offset', pi/18, 'qlim', [15 -182.5]*deg);
L(2) = Link('a', 3.6, 'alpha', -(pi/2) , 'd', 0.525, 'offset', pi/18, 'qlim', [182.5 -15]*deg);
L(3) = Link('a', 0 , 'alpha', 0, 'd', 0.45, 'offset', 0, 'qlim', [0, pi]);
% serial link will be used to help create the robot 
handles.robot = SerialLink(L, 'name', 'robot');
% robotic base that is 8 inches in y direction from universial coordinate 
% to robotic base, similarly for the tool tip  
handles.robot.base = [1 0 0 0; 0 1 0 8; 0 0 1 0; 0 0 0 1];
handles.robot.tool = [1 0 0 0; 0 1 0 0;0 0 1 -2.5; 0 0 0 1];
% below code will plot our robot with the theta values and update
% the floor of our robot with the 'workspace' command 
handles.robot.plot (handles.theta, 'workspace', W);

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in Reset_fn.
function Reset_fn_Callback(hObject, eventdata, handles)

% reset the robot back to starting postion by making theta values 0
handles.robot.plot ([0 0 0]);


% --- Executes on button press in Key_press.
function Key_press_Callback(hObject, eventdata, handles)

% ignore this function 


% --- Executes on key press with focus on Key_press and none of its controls.
function Key_press_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to Key_press (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles structure with handles and user data (see GUIDATA)

% this code will get the current graphic figure
f= gcf;

% m matrix specfies the axis (x,y,and z) and if there are any yaw, pitch, and row
m = [1 1 0 0 0 0];

% val will get the current arrow key from the keyboard, note computer can
% only recongnize numbers so the ascii table assigns a number to each
% keyboard key and this number is what is stored in the val variable

val=double(get(f,'CurrentCharacter'));

%do a forward kinematics
T = handles.robot.fkine([handles.theta(1), handles.theta(2), handles.theta(3)]);
 
% depending on the val value it will run the switch case based on it.
switch (val)

    case 28 % right
        
        T.t(1) = T.t(1) + .5;
        
    case 29  %left
        
        T.t(1) = T.t(1) - .5;
        
    case 30 %up
        
        T.t(2) = T.t(2) + .5;
        
    case 31  %down
        
        T.t(2) = T.t(2) - .5;
        
    otherwise
        
        disp ('unrecognized key');
        
end

% inverse kinematics is used here to get the joint angles which are theta 1
% theta 2 and theta 3. 
q = handles.robot.ikine(T,'q0', handles.theta,'mask', m);

% if q is not empty meaning there is some value in it then we will plot the
% robot and store the new joint angles in our theta values. This is good to
% prevent self assignment and decrease the time complexity of our program. 
if ~isempty(q)
    handles.robot.plot([q(1) q(2) q(3)]);
    
    handles.theta(1) = q(1);
    
    handles.theta(2) = q(2);
    
    handles.theta(3) = q(3);
end
   
% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in Optimize_fn.
function Optimize_fn_Callback(hObject, eventdata, handles)

% we first initalize our dh parameters. 
dh_parameters = [3.65 0 1.75 pi/18 3.6 -(pi/2) 0.525 pi/18 0 0 0.45 0];

% This functions calls our script file function and passes our intalized
% dh_parameters as an argument to the function
objFunc(dh_parameters)
% The code below will display and error values for each iteration completed
options = optimset('PlotFcns',@optimplotfval); 
% This code will give us our optimized dh parameter based of the minimal
% error threshold obsered by fminsearch
new_dh = fminsearch(@(dh_parameters) objFunc(dh_parameters), dh_parameters, options)

% The code below is similar to the create robot function, but this time we
% are plotting our robot based on the new optimized set of dh parameters.
% value of our work space to change the floor titles
W = [-10 15 0 20 -5 5];
% Pick the correct axe to plot our robot 
axes(handles.axes1)
deg = pi/180;
L(1) = Link('a', new_dh(1),'alpha', new_dh(2), 'd', new_dh(3), 'offset', new_dh(4), 'qlim', [15 -182.5]*deg);
L(2) = Link('a', new_dh(5), 'alpha', new_dh(6) , 'd', new_dh(7), 'offset', new_dh(8), 'qlim', [182.5 -15]*deg);
L(3) = Link('a', new_dh(9) , 'alpha', new_dh(10), 'd', new_dh(11), 'offset', new_dh(12), 'qlim', [0, pi]);
handles.robot = SerialLink(L, 'name', 'robot optimized');
handles.robot.base = [1 0 0 0; 0 1 0 8; 0 0 1 0; 0 0 0 1];
handles.robot.tool = [1 0 0 0; 0 1 0 0;0 0 1 -2.5; 0 0 0 1];
handles.robot.plot (handles.theta, 'workspace', W);
% Update handles structure
guidata(hObject, handles);
