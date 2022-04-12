function varargout = ecg_gui(varargin)
% ECG_GUI MATLAB code for ecg_gui.fig
%      ECG_GUI, by itself, creates a new ECG_GUI or raises the existing
%      singleton*.
%
%      H = ECG_GUI returns the handle to a new ECG_GUI or the handle to
%      the existing singleton*.
%
%      ECG_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ECG_GUI.M with the given input arguments.
%
%      ECG_GUI('Property','Value',...) creates a new ECG_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ecg_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ecg_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ecg_gui

% Last Modified by GUIDE v2.5 12-Dec-2013 12:17:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ecg_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @ecg_gui_OutputFcn, ...
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


% --- Executes just before ecg_gui is made visible.
function ecg_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ecg_gui (see VARARGIN)

% Choose default command line output for ecg_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
global ecg
ecg=[];
% UIWAIT makes ecg_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ecg_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
clear all;
close ecg_gui;
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
global ecg time
op=get(handles.popupmenu2,'Value');
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2

%   Set filter order N

switch(op)
    case 1

    case 2        
        
    case 3
        %Chevyshev filter
    case 4
        %cauer filter
end

% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
op=get(handles.popupmenu1,'Value');
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
switch (op)
    case 1
        str='mitdb/100';
    case 2
        str='mitdb/112';
    case 3
        str='svdb/805';
    case 4
        str='nstdb/118e24';
end
set(handles.text1,'String',str);


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
global time ecg fs;
%Clear plot figure
cla();
% hObject    handle to pushbutton7 (see GCBO)
%--------------------------------------------------------------
%Esto hay que modificarlo para el rollo de los menus
str=get(handles.text1,'String');
%--------------------------------------------------------------
[time,ecg,fs]=ECG_Load(str);
ecg=normalyce_ecg(ecg);
length(ecg)
ecg=ecg';
time=time';


% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
global time ecg
%   PLOT ecg data vs Time

if (get(handles.checkbox1,'Value')==0)
    h1=subplot(1,1,1);
    cla;
    hold on;
    plot(time,ecg,'linewidth',1.5);
    title('ECG signal');
    xlabel('Time');
    ylabel('mV');
    grid on;
    set(h1,'XScale','linear');
    hold off;
elseif (get(handles.checkbox1,'Value')==1)
    h1=subplot(1,1,1);
    hold on;
    plot(time,ecg,'linewidth',1.5);
    title('ECG signal');
    xlabel('Time');
    ylabel('mV');
    grid on;
    set(h1,'XScale','linear');
    hold off;
end

% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
global time ecg FFT_ecg
set(handles.checkbox1,'Value',0);
%   Plot FFT of ECG data
cla;
h1=subplot(1,1,1);
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
FFT_ecg=normalyce_fft(ecg);
stem(time,FFT_ecg,'linewidth',1.5);
set(h1,'XScale','log');
grid on;
xlabel('Frequency (Hz)');
ylabel('Amplitude %');
title('FFT of ECG data');
hold off;



% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
global time ecg FFT_ecg
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.checkbox1,'Value',0);
cla;
FFT_ecg=normalyce_fft(ecg);
%   Plot ECG data series.
h1=subplot(211);
hold on;
plot(h1,time,ecg,'Linewidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Amplitude (mV)');
title('ECG data');
%Plot FFT data.
h2=subplot(212);
stem(h2,time,FFT_ecg,'LineWidth',1.5);
set(h2,'XScale','log');
grid on;
xlabel('Frequency (Hz)');
ylabel('Amplitude %');
title('FFT');

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%------------------------------------------------------------
%       Set invisible plot panel
set(handles.uipanel1,'Visible','off');


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%------------------------------------------------------------------
%       Set visible ECG info panel.
set(handles.uipanel5,'Visible','on');



% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%-------------------------------------------------------------
%       Set invisible property filter panel
set(handles.uipanel4,'Visible','off');

% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1


% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in listbox2.
function listbox2_Callback(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox2


% --- Executes during object creation, after setting all properties.
function listbox2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%-------------------------------------------------------------
%       Set visible feilter properties
set(handles.uipanel4,'Visible','on');


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%-------------------------------------------------------------
%       Set invisible ecg info panel
set(handles.uipanel5,'Visible','off');
set(handles.uipanel6,'Visible','on');


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%----------------------------------------------------------------
%   Set visible ecg data selector.
if (get(handles.radiobutton3,'Value')==1)
    set(handles.radiobutton3,'Value',0);
end
set(handles.uipanel1,'Visible','on');
set(handles.uipanel2,'Visible','on');
set(handles.uipanel3,'Visible','on');
set(handles.uipanel5,'Visible','on');
set(handles.uipanel6,'Visible','on');
set(handles.uipanel7,'Visible','on');
set(handles.uipanel9,'Visible','on');
set(handles.uipanel10,'Visible','off');
% Hint: get(hObject,'Value') returns toggle state of radiobutton2


% --- Executes on button press in radiobutton3.
function radiobutton3_Callback(hObject, eventdata, handles)
global time ecg
% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%----------------------------------------------------------------
%   Calculate idea ECG data.
 [time,ecg]= ideal_ecg();
 ecg=normalyce_ecg(ecg);
%----------------------------------------------------------------
%   Set visible panels
if (get(handles.radiobutton2,'Value')==1)
    set(handles.radiobutton2,'Value',0);
    set(handles.uipanel2,'Visible','off');
end
set(handles.uipanel1,'Visible','on');
set(handles.uipanel3,'Visible','on');
set(handles.uipanel5,'Visible','on');
set(handles.uipanel6,'Visible','on');
set(handles.uipanel7,'Visible','on');
set(handles.uipanel9,'Visible','on');
set(handles.uipanel10,'Visible','on');
set(handles.text10,'Visible','off');
set(handles.text11,'Visible','off');
% Hint: get(hObject,'Value') returns toggle state of radiobutton3

% Peaks between -0.2mV and -0.5mV
%loc_Q = min_locs(smoothECG(min_locs)>-0.5 & smoothECG(min_locs)<-0.2);
% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
global ecg time fs
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
w=get(handles.togglebutton2,'Value');
[loc_Q,loc_R,loc_S]=find_QRS(ecg,w);
%   Plot data.
h1=subplot(1,1,1);
cla(h1);
hold on;
title('ECG signal and QRS complex');
xlabel('Time');
ylabel('mV');
grid on;
set(h1,'XScale','linear');

plot(time,ecg,'LineWidth',1.5);

if (get(handles.togglebutton2,'Value')==1)
    plot(time(loc_Q),ecg(loc_Q),'gv','MarkerFaceColor','g');
    plot(time(loc_R),ecg(loc_R),'rv','MarkerFaceColor','r');
    plot(time(loc_S),ecg(loc_S),'k^','MarkerFaceColor','k');
    legend('ECG data','P peaks','R peaks','S peaks');
else 
    plot(time(loc_S),ecg(loc_S),'k^','MarkerFaceColor','k');
    plot(time(loc_R),ecg(loc_R),'rv','MarkerFaceColor','r');
    plot(time(loc_Q),ecg(loc_Q),'gv','MarkerFaceColor','g');   
    legend('ECG data','Q peaks','R peaks','S peaks');
end
%   Set info panel
set(handles.text10,'Visible','on');
set(handles.text11,'Visible','on');
set(handles.text11,'String',num2str(fs));
[T,beats_min]=info_panel(loc_Q,loc_R,loc_S,ecg,time);
set(handles.text9,'string',num2str(beats_min));
set(handles.text7,'string',num2str(1/T));
L=max([length(loc_Q) length(loc_R) length(loc_S)]);
set(handles.text5,'string',num2str(L));
% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


%###############################################################
%###        My functions
%###############################################################

%This function gets ecg data and normalyce it.
function [ecg]=normalyce_ecg(ecg)
Max_ecg=max(ecg);
for i=1:length(ecg);
    ecg(i)= ecg(i)/Max_ecg;
end

function [ft]=normalyce_fft(ecg)
ft=abs(fft(ecg));
Max_ft=max(ft);
for i=1:length(ft);
    ft(i)= ft(i)/Max_ft;
end

%This function search QRS complex.
function [loc_Q,loc_R,loc_S]=find_QRS(ecg,w)
if (w==1)
    %Locate R wave
    [~,loc_R] =findpeaks(ecg,'MinPeakHeight',mean(ecg)*1.5);
    %Locate S_wave
    %Invert ECG data to find the manximun. Now s peak.
    ECG_i = -ecg;
    min_peak_heigth=(-mean(ecg)*0.8);
    [~,loc_S] = findpeaks(ECG_i,'MinPeakHeight',min_peak_heigth);
    %Locate Q wave
    [~,min_locs] = findpeaks(ecg,'MinPeakHeight',mean(ecg));
    % Peaks between -0.2mV and -0.5mV
    loc_Q = min_locs(ecg(min_locs)>0.2 & ecg(min_locs)<0.5);
    
elseif (w==0)
    %smoothECG = sgolayfilt(ecg,5,21);
    smoothECG = sgolayfilt(ecg,7,21);
    
    %Find S wave
    ECG_i = -ecg;
    [~,loc_S] = findpeaks(ECG_i,'MinPeakHeight',0.5,...
                                        'MinPeakDistance',200);
    %Fin R wave
    [~,loc_R] = findpeaks(ecg,'MinPeakHeight',0.5,...
                                    'MinPeakDistance',200);
	%Find Q wave
    [~,min_locs] = findpeaks(-smoothECG,'MinPeakDistance',40);
    % Peaks between -0.2mV and -0.5mV
    loc_Q = min_locs(smoothECG(min_locs)>-0.5 & smoothECG(min_locs)<-0.42);
    %Find P wave
    [~,min_locs] = findpeaks(smoothECG,'MinPeakDistance',40);
    % Peaks between -0.2mV and -0.5mV
    loc_P = min_locs(smoothECG(min_locs)>-0.5 & smoothECG(min_locs)<-0.42);
end
    

function [T,beats_min]=info_panel(loc_Q,loc_R,loc_S,ecg,time)
%Calculare signal period
T1=time(loc_R(2))-time(loc_R(2))/length(loc_R);
T2=time(loc_Q(2))-time(loc_Q(1))/length(loc_Q);
T3=time(loc_S(2))-time(loc_S(1))/length(loc_S);
T=round((T1+T2+T3)/3);
beats_min=60/T;

function rep(pq,qs,qrss,ss,ts,us,oe,la,anim)
global ecg_model;
la=30/la;
time=0.01:0.01:2;

p_w=pwave(time,pq(1),pq(2),pq(3),la);
q_w=pwave(time,qs(1),qs(2),qs(3),la);
qrs_w=qrswave(time,qrss(1),qrss(2),la);
s_w=swave(time,ss(1),ss(2),ss(3),la);
t_w=swave(time,ts(1),ts(2),ts(3),la);
u_w=swave(time,us(1),us(2),us(3),la);
ecg_model=p_w+qrs_w-t_w+s_w+q_w+u_w;

%Plot sequence
if (anim==1)
    comet1(oe,time,ecg_model);
elseif (anim==0)
    plot(oe,time,ecg_model,'Linewidth',2);
    axis(oe,[0,2,-1,2]);
    grid(oe,'on');
end

    
%#########################################################################
%#########################################################################
% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
global time ecg
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if (get(handles.checkbox1,'Value')==1)
    h1=subplot(1,1,1);
    hold on;
    plot(time,ecg,'ro');
    title('ECG signal');
    xlabel('Time');
    ylabel('mV');
    grid on;
    set(h1,'XScale','linear');
    hold off;
elseif (get(handles.checkbox1,'Value')==0)
    h1=subplot(1,1,1);
    cla;
    hold on;
    plot(time,ecg,'linewidth',1.5);
    title('ECG signal');
    xlabel('Time');
    ylabel('mV');
    grid on;
    set(h1,'XScale','linear');
    hold off;
end

% Hint: get(hObject,'Value') returns toggle state of checkbox1


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



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


% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
info_gui;
uiwait;



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


% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
global time ecg ecg_f
% hObject    handle to pushbutton19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Get filter type.
filt=get(handles.popupmenu2,'Value');
%Get filter order and check it.
order=str2num(get(handles.edit2,'String'));
if (order>10)
    order=10;
    set(handles.edit2,'String',num2str(order));
elseif (order<=0)
    order=1;
    set(handles.edit2,'String',num2str(order));
end
%Get number of filter
num=str2num(get(handles.edit1,'String'));
ecg_f=zeros(1,length(ecg));
%Get cutoff frequency
[w0,OK]=str2num(get(handles.edit3,'String'));
if(OK==0)
     TF=normalyce_fft(ecg);
    TF=TF(1,1:length(TF)/2);
    w0=findpeaks(TF,'Npeaks',num);
else
    if (w0>=1)
        w0=0.99;
    elseif (w0<=0)
        w0=0.01;
    end
end
set(handles.text18,'String',num2str(w0));
%Apply filter
switch (filt)
    case 1
    case 2
        ecg_f=ecg;
        for i=1:num
            %Butterworth filter
            [num,den]=butter(order,w0(i),'low');
            ecg_f=filter(num,den,ecg_f);            
        end
            ecg_f=normalyce_ecg(ecg_f);
            h=subplot(1,1,1);
            plot_filter(h,ecg,ecg_f,time);
    case 3
        ecg_f=ecg;
        for i=1:num
            %Chebyshev filter
            [num,den]=cheby1(order,0.5,w0(i),'low');
            ecg_f=filter(num,den,ecg_f);            
        end
            ecg_f=normalyce_ecg(ecg_f);
            h=subplot(1,1,1);
            plot_filter(h,ecg,ecg_f,time);
    case 4
        ecg_f=ecg;
        for i=1:num
            %Cauer filter
            [num,den]=ellip(order,0.5,20,w0(i),'low');
            ecg_f=filter(num,den,ecg_f);            
        end
            ecg_f=normalyce_ecg(ecg_f);
            h=subplot(1,1,1);
            plot_filter(h,ecg,ecg_f,time);
end

function plot_filter(h,ecg,ecg_f,time)
        cla(h);
        hold on,
        plot(h,time,ecg,'linewidth',1.5);
        plot(h,time,ecg_f,'r','linewidth',1.5);
        grid on;
        legend('ECG data','Filtered ECG','Location','Best');
        legend boxon;


% --- Executes on button press in pushbutton20.
function pushbutton20_Callback(hObject, eventdata, handles)
global ecg ecg_f
ecg=ecg_f;
% hObject    handle to pushbutton20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in togglebutton1.
function togglebutton1_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if (get(handles.togglebutton2,'Value')==1)
    set(handles.togglebutton2,'Value',0);
    set(handles.uipanel2,'Visible','off');
end
if (get(handles.togglebutton3,'Value')==1)
    set(handles.togglebutton3,'Value',0);
    set(handles.uipanel11,'Visible','off');
end
set(handles.uipanel1,'Visible','on');
set(handles.uipanel2,'Visible','on');
set(handles.uipanel3,'Visible','on');
set(handles.uipanel5,'Visible','on');
set(handles.uipanel6,'Visible','on');
set(handles.uipanel7,'Visible','on');
set(handles.uipanel9,'Visible','on');
% Hint: get(hObject,'Value') returns toggle state of togglebutton1


% --- Executes on button press in togglebutton2.
function togglebutton2_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global time ecg

% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%----------------------------------------------------------------
%   Calculate idea ECG data.
 
if (isempty(ecg)==1)
    [time,ecg]= ideal_ecg();
    ecg=normalyce_ecg(ecg);
end
%----------------------------------------------------------------
%   Set visible panels
if (get(handles.togglebutton1,'Value')==1)
    set(handles.togglebutton1,'Value',0);
    set(handles.uipanel2,'Visible','off');
end
if (get(handles.togglebutton3,'Value')==1)
    set(handles.togglebutton3,'Value',0);
    set(handles.uipanel11,'Visible','off');
end
set(handles.uipanel1,'Visible','on');
set(handles.uipanel3,'Visible','on');
set(handles.uipanel5,'Visible','on');
set(handles.uipanel6,'Visible','on');
set(handles.uipanel7,'Visible','on');
set(handles.uipanel9,'Visible','on');
set(handles.text10,'Visible','off');
set(handles.text11,'Visible','off');
% Hint: get(hObject,'Value') returns toggle state of togglebutton2


% --- Executes on button press in togglebutton3.
function togglebutton3_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to togglebutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
oe=handles.axes3;

if (get(handles.togglebutton1,'Value')==1)
    set(handles.togglebutton1,'Value',0);
    set(handles.uipanel2,'Visible','off');
end
if (get(handles.togglebutton2,'Value')==1)
    set(handles.togglebutton2,'Value',0);
end
set(handles.uipanel11,'Visible','on');

%Hide other panels
set(handles.uipanel1,'Visible','off');
set(handles.uipanel3,'Visible','off');
set(handles.uipanel5,'Visible','off');
set(handles.uipanel6,'Visible','off');
set(handles.uipanel7,'Visible','off');
set(handles.uipanel9,'Visible','off');

%Set default value of slider
%P wave
    set(handles.slider2,'Value',0.25);
    set(handles.slider5,'Value',0.09);
    set(handles.slider6,'Value',0.16);
	set(handles.text23,'String',num2str(get(handles.slider2,'Value')));
    set(handles.text24,'String',num2str(get(handles.slider5,'Value')));
    set(handles.text25,'String',num2str(get(handles.slider6,'Value')));
%Q wave
    set(handles.slider7,'Value',0.02);
    set(handles.slider8,'Value',0.06);
    set(handles.text29,'String',num2str(get(handles.slider7,'Value')));
    set(handles.text30,'String',num2str(get(handles.slider8,'Value')));
%QRS complex
    set(handles.slider12,'Value',1.6);
    set(handles.slider11,'Value',0.11);
    set(handles.text34,'String',num2str(get(handles.slider12,'Value')));
    set(handles.text33,'String',num2str(get(handles.slider11,'Value')));
%S wave
    set(handles.slider15,'Value',0.25);
    set(handles.slider14,'Value',0.066);
    set(handles.text39,'String',num2str(get(handles.slider14,'Value')));
    set(handles.text40,'String',num2str(get(handles.slider15,'Value')));
%T wave
    set(handles.slider18,'Value',0.35);
    set(handles.slider17,'Value',0.14);
    set(handles.slider16,'Value',0.2);
    set(handles.text44,'String',num2str(get(handles.slider16,'Value')));
    set(handles.text45,'String',num2str(get(handles.slider17,'Value')));
    set(handles.text46,'String',num2str(get(handles.slider18,'Value')));
%U wave
    set(handles.slider21,'Value',0.03);
    set(handles.slider20,'Value',0.04);
    set(handles.text51,'String',num2str(get(handles.slider20,'Value')));
    set(handles.text52,'String',num2str(get(handles.slider21,'Value')));
%Set heart beat rate
    set(handles.edit4,'String',num2str(72));

%Set value of ECG specifications
%   P-wave 
    a=get(handles.slider2,'Value');
    d=get(handles.slider5,'Value');
    PR_int=get(handles.slider6,'Value');
    ps=[a d PR_int];
%   Q wave
    a=get(handles.slider7,'Value');
    d=get(handles.slider8,'Value');
    qs=[a d 0.166];
%   QRS complex
    a=get(handles.slider12,'Value');
    d=get(handles.slider11,'Value');
    qrss=[a d 0];
%   S wave
    a=get(handles.slider15,'Value');
    d=get(handles.slider14,'Value');
    ss=[a d 0.09];
%   T wave
    a=get(handles.slider18,'Value');
    d=get(handles.slider17,'Value');
    ST_int=get(handles.slider16,'Value');
    ts=[a d ST_int];
%   U wave
    a=get(handles.slider21,'Value');
    d=get(handles.slider20,'Value');
    us=[a d 0.433];
%Get heart beat rate:
    la=str2num(get(handles.edit4,'String'));
%Set animate: FALSE
    set(handles.checkbox2,'Value',0);
    anim=get(handles.checkbox2,'Value');
%   Plot default values:
    rep(ps,qs,qrss,ss,ts,us,oe,la,anim);

    
% Hint: get(hObject,'Value') returns toggle state of togglebutton3



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


% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
global ecg ecg_model time
ecg=ecg_model;
time=0.01:0.01:2;

% hObject    handle to pushbutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la

% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
%Set new value in textbox
set(handles.text23,'String',num2str(get(handles.slider2,'Value')));

a=get(handles.slider2,'Value');
d=get(handles.slider5,'Value');
PR_int=get(handles.slider6,'Value');
ps=[a d PR_int];
anim=get(handles.checkbox2,'Value');
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);


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
function slider5_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text24,'String',num2str(get(handles.slider5,'Value')));
a=get(handles.slider2,'Value');
d=get(handles.slider5,'Value');
anim=get(handles.checkbox2,'Value');
PR_int=get(handles.slider6,'Value');
ps=[a d PR_int];
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);


% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text25,'String',num2str(get(handles.slider6,'Value')));
%P wave specifications
a=get(handles.slider2,'Value');
anim=get(handles.checkbox2,'Value');
d=get(handles.slider5,'Value');
PR_int=get(handles.slider6,'Value');
ps=[a d PR_int];
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);



% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider21_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to slider21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text52,'String',num2str(get(handles.slider21,'Value')));
anim=get(handles.checkbox2,'Value');
a=get(handles.slider21,'Value');
d=get(handles.slider20,'Value');

us=[a d 0.433];
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);


% --- Executes during object creation, after setting all properties.
function slider21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider20_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to slider20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text51,'String',num2str(get(handles.slider20,'Value')));
anim=get(handles.checkbox2,'Value');
a=get(handles.slider21,'Value');
d=get(handles.slider20,'Value');

us=[a d 0.433];
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);


% --- Executes during object creation, after setting all properties.
function slider20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider19_Callback(hObject, eventdata, handles)
% hObject    handle to slider19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider19_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider18_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to slider18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text46,'String',num2str(get(handles.slider18,'Value')));
anim=get(handles.checkbox2,'Value');
a=get(handles.slider18,'Value');
d=get(handles.slider17,'Value');
ST_int=get(handles.slider16,'Value');
ts=[a d ST_int];
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);



% --- Executes during object creation, after setting all properties.
function slider18_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider17_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to slider17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text45,'String',num2str(get(handles.slider17,'Value')));
anim=get(handles.checkbox2,'Value');
a=get(handles.slider18,'Value');
d=get(handles.slider17,'Value');
ST_int=get(handles.slider16,'Value');
ts=[a d ST_int];
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);


% --- Executes during object creation, after setting all properties.
function slider17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider16_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to slider16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text44,'String',num2str(get(handles.slider16,'Value')));
anim=get(handles.checkbox2,'Value');
a=get(handles.slider18,'Value');
d=get(handles.slider17,'Value');
ST_int=get(handles.slider16,'Value');
ts=[a d ST_int];
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);


% --- Executes during object creation, after setting all properties.
function slider16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider15_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to slider15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text40,'String',num2str(get(handles.slider15,'Value')));
anim=get(handles.checkbox2,'Value');
a=get(handles.slider15,'Value');
d=get(handles.slider14,'Value');

ss=[a d 0.09];
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);


% --- Executes during object creation, after setting all properties.
function slider15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider14_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to slider14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text39,'String',num2str(get(handles.slider14,'Value')));
anim=get(handles.checkbox2,'Value');
a=get(handles.slider15,'Value');
d=get(handles.slider14,'Value');

ss=[a d 0.09];
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);


% --- Executes during object creation, after setting all properties.
function slider14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider13_Callback(hObject, eventdata, handles)
% hObject    handle to slider13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider12_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to slider12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text34,'String',num2str(get(handles.slider12,'Value')));
anim=get(handles.checkbox2,'Value');
a=get(handles.slider12,'Value');
d=get(handles.slider11,'Value');

qrss=[a d 0];
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);



% --- Executes during object creation, after setting all properties.
function slider12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider11_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text33,'String',num2str(get(handles.slider11,'Value')));
anim=get(handles.checkbox2,'Value');
a=get(handles.slider12,'Value');
d=get(handles.slider11,'Value');

qrss=[a d 0];
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);



% --- Executes during object creation, after setting all properties.
function slider11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider10_Callback(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider9_Callback(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider8_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text30,'String',num2str(get(handles.slider8,'Value')));
anim=get(handles.checkbox2,'Value');
a=get(handles.slider7,'Value');
d=get(handles.slider8,'Value');

qs=[a d 0.166];
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);


% --- Executes during object creation, after setting all properties.
function slider8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider7_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.text29,'String',num2str(get(handles.slider7,'Value')));
anim=get(handles.checkbox2,'Value');
a=get(handles.slider7,'Value');
d=get(handles.slider8,'Value');

qs=[a d 0.166];
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);


% --- Executes during object creation, after setting all properties.
function slider7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton22.
function pushbutton22_Callback(hObject, eventdata, handles)

% hObject    handle to pushbutton22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%Set default value of slider
global ps qs qrss ss ts us oe la
%P wave
    set(handles.slider2,'Value',0.25);
    set(handles.slider5,'Value',0.09);
    set(handles.slider6,'Value',0.16);
	set(handles.text23,'String',num2str(get(handles.slider2,'Value')));
    set(handles.text24,'String',num2str(get(handles.slider5,'Value')));
    set(handles.text25,'String',num2str(get(handles.slider6,'Value')));
%Q wave
    set(handles.slider7,'Value',0.02);
    set(handles.slider8,'Value',0.06);
    set(handles.text29,'String',num2str(get(handles.slider7,'Value')));
    set(handles.text30,'String',num2str(get(handles.slider8,'Value')));
%QRS complex
    set(handles.slider12,'Value',1.6);
    set(handles.slider11,'Value',0.11);
    set(handles.text34,'String',num2str(get(handles.slider12,'Value')));
    set(handles.text33,'String',num2str(get(handles.slider11,'Value')));
%S wave
    set(handles.slider15,'Value',0.25);
    set(handles.slider14,'Value',0.06);
    set(handles.text39,'String',num2str(get(handles.slider14,'Value')));
    set(handles.text40,'String',num2str(get(handles.slider15,'Value')));
%T wave
    set(handles.slider18,'Value',0.35);
    set(handles.slider17,'Value',0.14);
    set(handles.slider16,'Value',0.2);
    set(handles.text44,'String',num2str(get(handles.slider16,'Value')));
    set(handles.text45,'String',num2str(get(handles.slider17,'Value')));
    set(handles.text46,'String',num2str(get(handles.slider18,'Value')));
%U wave
    set(handles.slider21,'Value',0.03);
    set(handles.slider20,'Value',0.04);
    set(handles.text51,'String',num2str(get(handles.slider20,'Value')));
    set(handles.text52,'String',num2str(get(handles.slider21,'Value')));
    %Set value of ECG specifications
%   P-wave 
    a=get(handles.slider2,'Value');
    d=get(handles.slider5,'Value');
    PR_int=get(handles.slider6,'Value');
    ps=[a d PR_int];
%   Q wave
    a=get(handles.slider7,'Value');
    d=get(handles.slider8,'Value');
    qs=[a d 0.166];
%   QRS complex
    a=get(handles.slider12,'Value');
    d=get(handles.slider11,'Value');
    qrss=[a d 0];
%   S wave
    a=get(handles.slider15,'Value');
    d=get(handles.slider14,'Value');
    ss=[a d 0.09];
%   T wave
    a=get(handles.slider18,'Value');
    d=get(handles.slider17,'Value');
    ST_int=get(handles.slider16,'Value');
    ts=[a d ST_int];
%   U wave
    a=get(handles.slider21,'Value');
    d=get(handles.slider20,'Value');
    us=[a d 0.433];
%Heart beat rate
    set(handles.edit4,'String',num2str(72));
    la=str2num(get(handles.edit4,'String'));
%   Plot default values:
anim=get(handles.checkbox2,'Value');
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);



% --- Executes on button press in pushbutton23.
function pushbutton23_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la 
% hObject    handle to pushbutton23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
la=str2num(get(handles.edit4,'String'));
anim=get(handles.checkbox2,'Value');
rep(ps,qs,qrss,ss,ts,us,oe,la,anim);



% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
global ecg
% hObject    handle to pushbutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ecg=[];


% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
global ps qs qrss ss ts us oe la
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
op=get(handles.popupmenu4,'Value')
switch(op)
    case 1        
%P wave
    set(handles.slider2,'Value',0.25);
    set(handles.slider5,'Value',0.09);
    set(handles.slider6,'Value',0.16);
	set(handles.text23,'String',num2str(get(handles.slider2,'Value')));
    set(handles.text24,'String',num2str(get(handles.slider5,'Value')));
    set(handles.text25,'String',num2str(get(handles.slider6,'Value')));
%Q wave
    set(handles.slider7,'Value',0.02);
    set(handles.slider8,'Value',0.06);
    set(handles.text29,'String',num2str(get(handles.slider7,'Value')));
    set(handles.text30,'String',num2str(get(handles.slider8,'Value')));
%QRS complex
    set(handles.slider12,'Value',1.6);
    set(handles.slider11,'Value',0.11);
    set(handles.text34,'String',num2str(get(handles.slider12,'Value')));
    set(handles.text33,'String',num2str(get(handles.slider11,'Value')));
%S wave
    set(handles.slider15,'Value',0.25);
    set(handles.slider14,'Value',0.06);
    set(handles.text39,'String',num2str(get(handles.slider14,'Value')));
    set(handles.text40,'String',num2str(get(handles.slider15,'Value')));
%T wave
    set(handles.slider18,'Value',0.35);
    set(handles.slider17,'Value',0.14);
    set(handles.slider16,'Value',0.2);
    set(handles.text44,'String',num2str(get(handles.slider16,'Value')));
    set(handles.text45,'String',num2str(get(handles.slider17,'Value')));
    set(handles.text46,'String',num2str(get(handles.slider18,'Value')));
%U wave
    set(handles.slider21,'Value',0.03);
    set(handles.slider20,'Value',0.04);
    set(handles.text51,'String',num2str(get(handles.slider20,'Value')));
    set(handles.text52,'String',num2str(get(handles.slider21,'Value')));
    %Set value of ECG specifications
%   P-wave 
    a=get(handles.slider2,'Value');
    d=get(handles.slider5,'Value');
    PR_int=get(handles.slider6,'Value');
    ps=[a d PR_int];
%   Q wave
    a=get(handles.slider7,'Value');
    d=get(handles.slider8,'Value');
    qs=[a d 0.166];
%   QRS complex
    a=get(handles.slider12,'Value');
    d=get(handles.slider11,'Value');
    qrss=[a d 0];
%   S wave
    a=get(handles.slider15,'Value');
    d=get(handles.slider14,'Value');
    ss=[a d 0.09];
%   T wave
    a=get(handles.slider18,'Value');
    d=get(handles.slider17,'Value');
    ST_int=get(handles.slider16,'Value');
    ts=[a d ST_int];
%   U wave
    a=get(handles.slider21,'Value');
    d=get(handles.slider20,'Value');
    us=[a d 0.433];
%Heart beat rate
    set(handles.edit4,'String',num2str(72));
    la=str2num(get(handles.edit4,'String'));
    anim=get(handles.checkbox2,'Value');
%   Plot default values:
    rep(ps,qs,qrss,ss,ts,us,oe,la,anim);
        
    case 2
%P wave
    set(handles.slider2,'Value',0.25);
    set(handles.slider5,'Value',0.09);
    set(handles.slider6,'Value',0.16);
	set(handles.text23,'String',num2str(get(handles.slider2,'Value')));
    set(handles.text24,'String',num2str(get(handles.slider5,'Value')));
    set(handles.text25,'String',num2str(get(handles.slider6,'Value')));
%Q wave
    set(handles.slider7,'Value',0.02);
    set(handles.slider8,'Value',0.01);
    set(handles.text29,'String',num2str(get(handles.slider7,'Value')));
    set(handles.text30,'String',num2str(get(handles.slider8,'Value')));
%QRS complex
    set(handles.slider12,'Value',1.72);
    set(handles.slider11,'Value',0.04);
    set(handles.text34,'String',num2str(get(handles.slider12,'Value')));
    set(handles.text33,'String',num2str(get(handles.slider11,'Value')));
%S wave
    set(handles.slider15,'Value',0.25);
    set(handles.slider14,'Value',0.06);
    set(handles.text39,'String',num2str(get(handles.slider14,'Value')));
    set(handles.text40,'String',num2str(get(handles.slider15,'Value')));
%T wave
    set(handles.slider18,'Value',0.35);
    set(handles.slider17,'Value',0.14);
    set(handles.slider16,'Value',0.2);
    set(handles.text44,'String',num2str(get(handles.slider16,'Value')));
    set(handles.text45,'String',num2str(get(handles.slider17,'Value')));
    set(handles.text46,'String',num2str(get(handles.slider18,'Value')));
%U wave
    set(handles.slider21,'Value',0.03);
    set(handles.slider20,'Value',0.04);
    set(handles.text51,'String',num2str(get(handles.slider20,'Value')));
    set(handles.text52,'String',num2str(get(handles.slider21,'Value')));
    %Set value of ECG specifications
%   P-wave 
    a=get(handles.slider2,'Value');
    d=get(handles.slider5,'Value');
    PR_int=get(handles.slider6,'Value');
    ps=[a d PR_int];
%   Q wave
    a=get(handles.slider7,'Value');
    d=get(handles.slider8,'Value');
    qs=[a d 0.166];
%   QRS complex
    a=get(handles.slider12,'Value');
    d=get(handles.slider11,'Value');
    qrss=[a d 0];
%   S wave
    a=get(handles.slider15,'Value');
    d=get(handles.slider14,'Value');
    ss=[a d 0.09];
%   T wave
    a=get(handles.slider18,'Value');
    d=get(handles.slider17,'Value');
    ST_int=get(handles.slider16,'Value');
    ts=[a d ST_int];
%   U wave
    a=get(handles.slider21,'Value');
    d=get(handles.slider20,'Value');
    us=[a d 0.433];
%Heart beat rate
    la=str2num(get(handles.edit4,'String'));
    anim=get(handles.checkbox2,'Value');
%   Plot default values:
    rep(ps,qs,qrss,ss,ts,us,oe,la,anim);
        
    case 3
%P wave
    set(handles.slider2,'Value',0.19);
    set(handles.slider5,'Value',0.41);
    set(handles.slider6,'Value',0.17);
	set(handles.text23,'String',num2str(get(handles.slider2,'Value')));
    set(handles.text24,'String',num2str(get(handles.slider5,'Value')));
    set(handles.text25,'String',num2str(get(handles.slider6,'Value')));
%Q wave
    set(handles.slider7,'Value',0.02);
    set(handles.slider8,'Value',0.06);
    set(handles.text29,'String',num2str(get(handles.slider7,'Value')));
    set(handles.text30,'String',num2str(get(handles.slider8,'Value')));
%QRS complex
    set(handles.slider12,'Value',1.6);
    set(handles.slider11,'Value',0.12);
    set(handles.text34,'String',num2str(get(handles.slider12,'Value')));
    set(handles.text33,'String',num2str(get(handles.slider11,'Value')));
%S wave
    set(handles.slider15,'Value',0.25);
    set(handles.slider14,'Value',0.03);
    set(handles.text39,'String',num2str(get(handles.slider14,'Value')));
    set(handles.text40,'String',num2str(get(handles.slider15,'Value')));
%T wave
    set(handles.slider18,'Value',0.1);
    set(handles.slider17,'Value',0.33);
    set(handles.slider16,'Value',0.22);
    set(handles.text44,'String',num2str(get(handles.slider16,'Value')));
    set(handles.text45,'String',num2str(get(handles.slider17,'Value')));
    set(handles.text46,'String',num2str(get(handles.slider18,'Value')));
%U wave
    set(handles.slider21,'Value',0.00);
    set(handles.slider20,'Value',0.03);
    set(handles.text51,'String',num2str(get(handles.slider20,'Value')));
    set(handles.text52,'String',num2str(get(handles.slider21,'Value')));
    %Set value of ECG specifications
%   P-wave 
    a=get(handles.slider2,'Value');
    d=get(handles.slider5,'Value');
    PR_int=get(handles.slider6,'Value');
    ps=[a d PR_int];
%   Q wave
    a=get(handles.slider7,'Value');
    d=get(handles.slider8,'Value');
    qs=[a d 0.166];
%   QRS complex
    a=get(handles.slider12,'Value');
    d=get(handles.slider11,'Value');
    qrss=[a d 0];
%   S wave
    a=get(handles.slider15,'Value');
    d=get(handles.slider14,'Value');
    ss=[a d 0.09];
%   T wave
    a=get(handles.slider18,'Value');
    d=get(handles.slider17,'Value');
    ST_int=get(handles.slider16,'Value');
    ts=[a d ST_int];
%   U wave
    a=get(handles.slider21,'Value');
    d=get(handles.slider20,'Value');
    us=[a d 0.433];
%Heart beat rate
    set(handles.edit4,'String',num2str(72));
    la=str2num(get(handles.edit4,'String'));
    anim=get(handles.checkbox2,'Value');
%   Plot default values:
    rep(ps,qs,qrss,ss,ts,us,oe,la,anim);
end
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4


% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
