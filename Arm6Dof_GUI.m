function varargout = Arm6Dof_GUI(varargin)
% Arm6Dof_GUI MATLAB code for Arm6Dof_GUI.fig
%      Arm6Dof_GUI, by itself, creates a new Arm6Dof_GUI or raises the existing
%      singleton*.
%
%      H = Arm6Dof_GUI returns the handle to a new Arm6Dof_GUI or the handle to
%      the existing singleton*.
%
%      Arm6Dof_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in Arm6Dof_GUI.M with the given input arguments.
%
%      Arm6Dof_GUI('Property','Value',...) creates a new Arm6Dof_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Arm6Dof_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Arm6Dof_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Arm6Dof_GUI

% Last Modified by GUIDE v2.5 08-Jun-2019 09:10:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Arm6Dof_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Arm6Dof_GUI_OutputFcn, ...
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


% --- Executes just before Arm6Dof_GUI is made visible.
function Arm6Dof_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Arm6Dof_GUI (see VARARGIN)

% Choose default command line output for Arm6Dof_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Arm6Dof_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);
axes(handles.axes1);
image = imread('robot.jpg');
imshow(image);
axes(handles.axes2);
image = imread('weixin.png');
imshow(image);

% --- Outputs from this function are returned to the command line.
function varargout = Arm6Dof_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function Tpt = pos2tran(Xpt)
  % pos to homogeneous transformation matrix
  % first row
  Tpt(1 , 1) = cos(Xpt(4))*cos(Xpt(5))*cos(Xpt(6))-sin(Xpt(4))*sin(Xpt(6));
  Tpt(1 , 2) = -cos(Xpt(4))*cos(Xpt(5))*sin(Xpt(6))-sin(Xpt(4))*cos(Xpt(6));
  Tpt(1 , 3) = cos(Xpt(4))*sin(Xpt(5));
  Tpt(1 , 4) = Xpt(1);
  % second row
  Tpt(2 , 1) = sin(Xpt(4))*cos(Xpt(5))*cos(Xpt(6))+cos(Xpt(4))*sin(Xpt(6));
  Tpt(2 , 2) = -sin(Xpt(4))*cos(Xpt(5))*sin(Xpt(6))+cos(Xpt(4))*cos(Xpt(6));
  Tpt(2 , 3) = sin(Xpt(4))*sin(Xpt(5));
  Tpt(2 , 4) = Xpt(2);
  % third row
  Tpt(3 , 1) = -sin(Xpt(5))*cos(Xpt(6));
  Tpt(3 , 2) = sin(Xpt(5))*sin(Xpt(6));
  Tpt(3 , 3) = cos(Xpt(5));
  Tpt(3 , 4) = Xpt(3);
  % forth row
  Tpt(4 , 1) = 0.0;
  Tpt(4 , 2) = 0.0;
  Tpt(4 , 3) = 0.0;
  Tpt(4 , 4) = 1.0;

% 求齐次变换矩阵的逆
function Titf = invtran(Titi)
  % finding the inverse of the homogeneous transformation matrix
  % first row
  Titf(1 , 1) = Titi(1 , 1);
  Titf(1 , 2) = Titi(2 , 1);
  Titf(1 , 3) = Titi(3 , 1);
  Titf(1 , 4) = -Titi(1 , 1)*Titi(1 , 4)-Titi(2 , 1)*Titi(2 , 4)-Titi(3 , 1)*Titi(3 , 4);
  % second row
  Titf(2 , 1) = Titi(1 , 2);
  Titf(2 , 2) = Titi(2 , 2);
  Titf(2 , 3) = Titi(3 , 2);
  Titf(2 , 4) = -Titi(1 , 2)*Titi(1 , 4)-Titi(2 , 2)*Titi(2 , 4)-Titi(3 , 2)*Titi(3 , 4);
  % third row
  Titf(3 , 1) = Titi(1 , 3);
  Titf(3 , 2) = Titi(2 , 3);
  Titf(3 , 3) = Titi(3 , 3);
  Titf(3 , 4) = -Titi(1 , 3)*Titi(1 , 4)-Titi(2 , 3)*Titi(2 , 4)-Titi(3 , 3)*Titi(3 , 4);
  % forth row
  Titf(4 , 1) = 0.0;
  Titf(4 , 2) = 0.0;
  Titf(4 , 3) = 0.0;
  Titf(4 , 4) = 1.0;

function Tdh = DH1line(thetadh,alfadh,rdh,ddh)
  % creats Denavit-Hartenberg homogeneous transformation matrix
  % first row
  Tdh(1 , 1) = cos(thetadh);
  Tdh(1 , 2) = -sin(thetadh)*cos(alfadh);
  Tdh(1 , 3) = sin(thetadh)*sin(alfadh);
  Tdh(1 , 4) = rdh*cos(thetadh);
  % second row
  Tdh(2 , 1) = sin(thetadh);
  Tdh(2 , 2) = cos(thetadh)*cos(alfadh);
  Tdh(2 , 3) = -cos(thetadh)*sin(alfadh);
  Tdh(2 , 4) = rdh*sin(thetadh);
  % third row
  Tdh(3 , 1) = 0.0;
  Tdh(3 , 2) = sin(alfadh);
  Tdh(3 , 3) = cos(alfadh);
  Tdh(3 , 4) = ddh;
  % forth row
  Tdh(4 , 1) = 0.0;
  Tdh(4 , 2) = 0.0;
  Tdh(4 , 3) = 0.0;
  Tdh(4 , 4) = 1.0;

%齐次变换矩阵的位置
function Xtp = tran2pos(Ttp)
  % pos from homogeneous transformation matrix
  Xtp(1) = Ttp(1 , 4);
  Xtp(2) = Ttp(2 , 4);
  Xtp(3) = Ttp(3 , 4);
  Xtp(5) = atan2(sqrt(Ttp(3 , 1)*Ttp(3 , 1) + Ttp(3 , 2)*Ttp(3 , 2)),Ttp(3 , 3));
  Xtp(4) = atan2(Ttp(2 , 3)/sin(Xtp(5)),Ttp(1 , 3)/sin(Xtp(5)));
  Xtp(6) = atan2(Ttp(3 , 2)/sin(Xtp(5)),-Ttp(3 , 1)/sin(Xtp(5)));

function Xfk = ForwardK(Jfk)
  % forward kinematics
  % input: Jfk - joints value for the calculation of the forward kinematics
  % output: Xfk - pos value for the calculation of the forward kinematics
  
  % Denavit-Hartenberg matrix
  theta=[Jfk(1); -90+Jfk(2); Jfk(3); Jfk(4); Jfk(5); Jfk(6)];
  alfa=[-90; 0; -90; 90; -90; 0];
  r=[47; 110; 26; 0; 0; 0];
  d=[133; 0; 7; 117.5; 0; 28];
  % from deg to rad
  theta=theta*pi/180;
  alfa=alfa*pi/180;
  
  % work frame
  Xwf=[0; 0; 0; 0; 0; 0];  
  % tool frame
  Xtf=[0; 0; 0; 0; 0; 0];  
  % work frame transformation matrix
  Twf=pos2tran(Xwf);  
  % tool frame transformation matrix
  Ttf=pos2tran(Xtf);
  
  % DH homogeneous transformation matrix
  T01=DH1line(theta(1),alfa(1),r(1),d(1));
  T12=DH1line(theta(2),alfa(2),r(2),d(2));
  T23=DH1line(theta(3),alfa(3),r(3),d(3));
  T34=DH1line(theta(4),alfa(4),r(4),d(4));
  T45=DH1line(theta(5),alfa(5),r(5),d(5));
  T56=DH1line(theta(6),alfa(6),r(6),d(6));

  Tw1=Twf*T01;
  Tw2=Tw1*T12;
  Tw3=Tw2*T23;
  Tw4=Tw3*T34;
  Tw5=Tw4*T45;
  Tw6=Tw5*T56;
  Twt=Tw6*Ttf;
  
  % calculate pos from transformation matrix
  Xfk=tran2pos(Twt);
  Xfk(4:6)=Xfk(4:6)/pi*180;
    
function Jik = InverseK(Xik)
  % inverse kinematics
  % input: Xik - pos value for the calculation of the inverse kinematics
  % output: Jfk - joints value for the calculation of the inversed kinematics
  
  % from deg to rad
  Xik(4:6)=Xik(4:6)*pi/180; 
  
  % Denavit-Hartenberg matrix
  theta=[0; -90+0; 0; 0; 0; 0];
  alfa=[-90; 0; -90; 90; -90; 0];
  r=[47; 110; 26; 0; 0; 0];
  d=[133; 0; 7; 117.5; 0; 28];
  % from deg to rad
  theta=theta*pi/180;
  alfa=alfa*pi/180;
  
  % work frame
  Xwf=[0; 0; 0; 0; 0; 0];
  % tool frame
  Xtf=[0; 0; 0; 0; 0; 0];
  
  % work frame transformation matrix
  Twf=pos2tran(Xwf);  
  % tool frame transformation matrix
  Ttf=pos2tran(Xtf); 
  % total transformation matrix
  Twt=pos2tran(Xik);
  
  % find T06
  inTwf=invtran(Twf);
  inTtf=invtran(Ttf);
  Tw6=Twt*inTtf;
  T06=inTwf*Tw6;
  
  % positon of the spherical wrist
  Xsw=T06(1:3,4)-d(6)*T06(1:3,3);  
  % joints variable
  Jik=zeros(6,1);
  % first joint
  Jik(1)=atan2(Xsw(2),Xsw(1))-atan2(d(3),sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2));
  % second joint
  Jik(2)=pi/2-acos((r(2)^2+(Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2-(r(3)^2+d(4)^2))/(2*r(2)*sqrt((Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)))-atan((Xsw(3)-d(1))/(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1)));
  % third joint
  Jik(3)=pi-acos((r(2)^2+r(3)^2+d(4)^2-(Xsw(3)-d(1))^2-(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)/(2*r(2)*sqrt(r(3)^2+d(4)^2)))-atan(d(4)/r(3));
  % last three joints
  T01=DH1line(theta(1)+Jik(1),alfa(1),r(1),d(1));
  T12=DH1line(theta(2)+Jik(2),alfa(2),r(2),d(2));
  T23=DH1line(theta(3)+Jik(3),alfa(3),r(3),d(3));
  T02=T01*T12;
  T03=T02*T23;
  inT03=invtran(T03);
  T36=inT03*T06;
  % forth joint
  Jik(4)=atan2(-T36(2,3),-T36(1,3));
  % fifth joint
  Jik(5)=atan2(sqrt(T36(1,3)^2+T36(2,3)^2),T36(3,3));
  % sixth joints
  Jik(6)=atan2(-T36(3,2),T36(3,1));
  % rad to deg
  Jik=Jik/pi*180;
    
function theta2pos(handles)
    ModelName = 'Arm6Dof';
    global var;   
    t(1)=get(handles.slider1,'value');
    set(handles.edit1,'string',num2str(t(1)));
    t(2)=get(handles.slider2,'value');
    set(handles.edit2,'string',num2str(t(2)));
    t(3)=get(handles.slider3,'value');
    set(handles.edit3,'string',num2str(t(3)));
    t(4)=get(handles.slider4,'value');
    set(handles.edit4,'string',num2str(t(4)));
    t(5)=get(handles.slider5,'value');
    set(handles.edit5,'string',num2str(t(5)));
    t(6)=get(handles.slider6,'value');
    set(handles.edit6,'string',num2str(t(6)));
    
    set_param([ModelName '/Slider Gain'],'Gain',num2str(t(1)))
    set_param([ModelName '/Slider Gain1'],'Gain',num2str(t(2)))
    set_param([ModelName '/Slider Gain2'],'Gain',num2str(t(3)))
    set_param([ModelName '/Slider Gain3'],'Gain',num2str(t(4)))
    set_param([ModelName '/Slider Gain4'],'Gain',num2str(t(5)))
    set_param([ModelName '/Slider Gain5'],'Gain',num2str(t(6)))
    
    pos = ForwardK(t);    
    
    set(handles.edit7,'string',num2str(pos(1)));
    set(handles.edit8,'string',num2str(pos(2)));
    set(handles.edit9,'string',num2str(pos(3)));

function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    theta2pos(handles);

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    theta2pos(handles);

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    theta2pos(handles);

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    theta2pos(handles);

% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    theta2pos(handles);

% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    theta2pos(handles);

% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    ModelName = 'Arm6Dof';
    global var;    
    t=[0;0;0;0;0;0];
    set(handles.slider1,'value',t(1));
    set(handles.slider2,'value',t(2));
    set(handles.slider3,'value',t(3));
    set(handles.slider4,'value',t(4));
    set(handles.slider5,'value',t(5));
    set(handles.slider6,'value',t(6));
    set(handles.edit1,'string',num2str(t(1)));    
    set(handles.edit2,'string',num2str(t(2)));    
    set(handles.edit3,'string',num2str(t(3)));
    set(handles.edit4,'string',num2str(t(4)));
    set(handles.edit5,'string',num2str(t(5)));
    set(handles.edit6,'string',num2str(t(6)));
    
    set_param([ModelName '/Slider Gain'],'Gain',num2str(t(1)))
    set_param([ModelName '/Slider Gain1'],'Gain',num2str(t(2)))
    set_param([ModelName '/Slider Gain2'],'Gain',num2str(t(3)))
    set_param([ModelName '/Slider Gain3'],'Gain',num2str(t(4)))
    set_param([ModelName '/Slider Gain4'],'Gain',num2str(t(5)))
    set_param([ModelName '/Slider Gain5'],'Gain',num2str(t(6)))
    
    pos = ForwardK(t);    
    
    set(handles.edit7,'string',num2str(pos(1)));
    set(handles.edit8,'string',num2str(pos(2)));
    set(handles.edit9,'string',num2str(pos(3)));    
    set(handles.slider7,'value',pos(1));
    set(handles.slider8,'value',pos(2));
    set(handles.slider9,'value',pos(3)); 

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    ModelName = 'Arm6Dof';
    global var;
    pos = [0 0 0 0 0 0];
    pos(1) = str2double(get(handles.edit7,'string'));
    set(handles.slider7,'value',pos(1));    
    pos(2) = str2double(get(handles.edit8,'string'));
    set(handles.slider8,'value',pos(2));    
    pos(3) = str2double(get(handles.edit9,'string'));
    set(handles.slider9,'value',pos(3));    
    
    t = InverseK(pos);   
    
    set(handles.slider1,'value',t(1));
    set(handles.slider2,'value',t(2));
    set(handles.slider3,'value',t(3));
    set(handles.slider4,'value',t(4));
    set(handles.slider5,'value',t(5));
    set(handles.slider6,'value',t(6));
    
    guidata(hObject,handles);
    set(handles.edit1,'string',num2str(t(1)));
    set(handles.edit2,'string',num2str(t(2)));
    set(handles.edit3,'string',num2str(t(3)));
    set(handles.edit4,'string',num2str(t(4)));
    set(handles.edit5,'string',num2str(t(5)));
    set(handles.edit6,'string',num2str(t(6)));
    
    set_param([ModelName '/Slider Gain'],'Gain',num2str(t(1)))
    set_param([ModelName '/Slider Gain1'],'Gain',num2str(t(2)))   
    set_param([ModelName '/Slider Gain2'],'Gain',num2str(t(3)))
    set_param([ModelName '/Slider Gain3'],'Gain',num2str(t(4)))
    set_param([ModelName '/Slider Gain4'],'Gain',num2str(t(5)))
    set_param([ModelName '/Slider Gain5'],'Gain',num2str(t(6)))

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    close;


function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider7_Callback(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider8_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider9_Callback(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
