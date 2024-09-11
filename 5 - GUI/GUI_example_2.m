function varargout = GUI_example_2(varargin)
% GUI_EXAMPLE_2 MATLAB code for GUI_example_2.fig
%      GUI_EXAMPLE_2, by itself, creates a new GUI_EXAMPLE_2 or raises the existing
%      singleton*.
%
%      H = GUI_EXAMPLE_2 returns the handle to a new GUI_EXAMPLE_2 or the handle to
%      the existing singleton*.
%
%      GUI_EXAMPLE_2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_EXAMPLE_2.M with the given input arguments.
%
%      GUI_EXAMPLE_2('Property','Value',...) creates a new GUI_EXAMPLE_2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_example_2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_example_2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_example_2

% Last Modified by GUIDE v2.5 10-Sep-2024 10:45:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_example_2_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_example_2_OutputFcn, ...
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


% --- Executes just before GUI_example_2 is made visible.
function GUI_example_2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_example_2 (see VARARGIN)

% Choose default command line output for GUI_example_2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI_example_2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% Code to be displayed by default
t = 0:0.01:10*pi;
y = 5*sin(t);
plot(t, y);
grid on;
axis([0 5*pi -5 5]);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_example_2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
temp1 = get(handles.slider1, 'Value');
set(handles.axes1, 'ylim', [-(5+temp1) 5+temp1]);


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
temp2 = get(handles.slider2, 'Value');
set(handles.axes1, 'xlim', [0 (5+temp2)*pi]);


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function Amp_Callback(hObject, eventdata, handles)
% hObject    handle to Amp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Amp as text
%        str2double(get(hObject,'String')) returns contents of Amp as a double


% --- Executes during object creation, after setting all properties.
function Amp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Amp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

Amp = str2double(get(handles.Amp, 'String'));
Freq = str2double(get(handles.Freq, 'String'));
PhaseShift = str2double(get(handles.PhaseShift, 'String'));
endTime = str2double(get(handles.endTime, 'String'));
count = str2double(get(handles.count, 'String'));
contents = cellstr(get(handles.targetFunc, 'String'));
target = contents{get(handles.targetFunc, 'Value')};
TargetFunc = str2func(target);

% Calculate the value of function y
t = 0:0.01:endTime;
y = Amp * TargetFunc(2*pi*Freq*t) + PhaseShift;


plot(handles.axes1, t, y);
grid on;

% set(handles.endTime, 'enable', 'off')
% if (count>=0)&&(count<=3)&&(Amp~=0)&&(Freq~=0)
%     errorCode = 1;
%     count = count + 1;
%     set(handles.count, 'String', count);
%     preEqu = get(handles.dispEqu, 'String');
%     if strcmp(preEqu, 'Nothing')
%     preEqu = '';
%     else
%     preEqu = eval([' '' ' preEqu ' + '' ']);
%     end
%     reEqu = eval([''' ' preEqu '' num2str(Amp) '*' TargetFunc '(2*pi*‘' ...
%         'num2str(Freq) '*t +' num2str(PhaseShift) ') ''']);
%     set(handles.dispEqu, 'String', reEqu);
%     t = 0:0.01:endTime;
%     y = subs(reEqu, 't', t, 0);
%     plot(handles.axes1, t, y)
%     set(handles.axes1, 'xlim', [0 endTime], 'ylim', [min(y) max(y)]*1.1, ...
%         'xgrid', 'on', 'ygrid', 'on');
% 
% elseif count>3
%     errorCode = 2;
% elseif Amp==0
%     errorCode = 3;
% elseif Freq==0
%     errorCode = 4;




function Freq_Callback(hObject, eventdata, handles)
% hObject    handle to Freq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Freq as text
%        str2double(get(hObject,'String')) returns contents of Freq as a double


% --- Executes during object creation, after setting all properties.
function Freq_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Freq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PhaseShift_Callback(hObject, eventdata, handles)
% hObject    handle to PhaseShift (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PhaseShift as text
%        str2double(get(hObject,'String')) returns contents of PhaseShift as a double


% --- Executes during object creation, after setting all properties.
function PhaseShift_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PhaseShift (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in targetFunc.
function targetFunc_Callback(hObject, eventdata, handles)
% hObject    handle to targetFunc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns targetFunc contents as cell array
%        contents{get(hObject,'Value')} returns selected item from targetFunc


% --- Executes during object creation, after setting all properties.
function targetFunc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to targetFunc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function endTime_Callback(hObject, eventdata, handles)
% hObject    handle to endTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of endTime as text
%        str2double(get(hObject,'String')) returns contents of endTime as a double


% --- Executes during object creation, after setting all properties.
function endTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to endTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function count_Callback(hObject, eventdata, handles)
% hObject    handle to count (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of count as text
%        str2double(get(hObject,'String')) returns contents of count as a double


% --- Executes during object creation, after setting all properties.
function count_CreateFcn(hObject, eventdata, handles)
% hObject    handle to count (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
