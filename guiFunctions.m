function varargout = teach(varargin)
% TEACH MATLAB code for teach.fig
%      TEACH, by itself, creates a new TEACH or raises the existing
%      singleton*.
%
%      H = TEACH returns the handle to a new TEACH or the handle to
%      the existing singleton*.
%
%      TEACH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TEACH.M with the given input arguments.
%
%      TEACH('Property','Value',...) creates a new TEACH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before teach_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to teach_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help teach

% Last Modified by GUIDE v2.5 11-Oct-2017 22:22:46

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @teach_OpeningFcn, ...
                   'gui_OutputFcn',  @teach_OutputFcn, ...
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

% --- Executes just before teach is made visible.
function teach_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to teach (see VARARGIN)

% Choose default command line output for teach
handles.cyton = Cyton;


function q7_value_callback(hObject,eventdata,handles)

