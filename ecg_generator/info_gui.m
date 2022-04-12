function varargout = info_gui(varargin)
% INFO_GUI MATLAB code for info_gui.fig
%      INFO_GUI, by itself, creates a new INFO_GUI or raises the existing
%      singleton*.
%
%      H = INFO_GUI returns the handle to a new INFO_GUI or the handle to
%      the existing singleton*.
%
%      INFO_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INFO_GUI.M with the given input arguments.
%
%      INFO_GUI('Property','Value',...) creates a new INFO_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before info_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to info_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help info_gui

% Last Modified by GUIDE v2.5 21-Nov-2013 13:12:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @info_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @info_gui_OutputFcn, ...
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


% --- Executes just before info_gui is made visible.
function info_gui_OpeningFcn(hObject, eventdata, handles, varargin)
global ecg ecg_f time
err=abs(ecg-ecg_f);
ME=sum(err)/length(ecg);
MSE=mean((err).^2);
RMSE = sqrt(mean((err).^2));
STD=std(err);
R2=(corrcoef(ecg,ecg_f))^2


set(handles.text8,'String',num2str(ME));
set(handles.text7,'String',num2str(MSE));
set(handles.text9,'String',num2str(STD));
set(handles.text10,'String',num2str(R2));
set(handles.text15,'String',num2str(RMSE));


fft_ecg=abs(fft(ecg));
fft_ecgf=abs(fft(ecg_f));

n=str2num(get(handles.text12,'String'));
[MSH_ecg,~]=findpeaks(fft_ecg,'Npeaks',n);
[MSH_ecgf,~]=findpeaks(fft_ecgf,'Npeaks',n);

set(handles.text13,'String',num2str(MSH_ecg));
set(handles.text16,'String',num2str(MSH_ecgf));

% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to info_gui (see VARARGIN)

% Choose default command line output for info_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes info_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = info_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close info_gui


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
