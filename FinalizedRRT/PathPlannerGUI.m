function PathPlannerGUI
f = figure('Visible','off','Position',[360,500,400,285]);
%% Prevents the box from scaling smaller than its initialization
figszfun = @(~,~) set(f, 'position', max([0 0 400 285], f.Position));
f.SizeChangedFcn = figszfun; %%
%% Starting Joints Text and Sliders
joint1Text = uicontrol('Style','edit','String','Joint 1',...
    'Position',[25,250,40,20],...
    'Callback',@joint1text_Callback);
joint1Slider = uicontrol('Style','slider','String','Joint 1',...
    'Position',[70,250,100,20],...
    'Callback',@joint1slider_Callback);
joint2Text = uicontrol('Style','edit','String','Joint 2',...
    'Position',[25,225,40,20],...
    'Callback',@joint2text_Callback);
joint2Slider = uicontrol('Style','slider','String','Joint 2',...
    'Position',[70,225,100,20],...
    'Callback',@joint2slider_Callback);
joint3Text = uicontrol('Style','edit','String','Joint 3',...
    'Position',[25,200,40,20],...
    'Callback',@joint3text_Callback);
joint3Slider = uicontrol('Style','slider','String','Joint 3',...
    'Position',[70,200,100,20],...
    'Callback',@joint3slider_Callback);
joint4Text = uicontrol('Style','edit','String','Joint 4',...
    'Position',[25,175,40,20],...
    'Callback',@joint4text_Callback);
joint4Slider = uicontrol('Style','slider','String','Joint 4',...
    'Position',[70,175,100,20],...
    'Callback',@joint4slider_Callback);
joint5Text = uicontrol('Style','edit','String','Joint 5',...
    'Position',[25,150,40,20],...
    'Callback',@joint5text_Callback);
joint5Slider = uicontrol('Style','slider','String','Joint 5',...
    'Position',[70,150,100,20],...
    'Callback',@joint5slider_Callback);
joint6Text = uicontrol('Style','edit','String','Joint 6',...
    'Position',[25,125,40,20],...
    'Callback',@joint6text_Callback);
joint6Slider = uicontrol('Style','slider','String','Joint 6',...
    'Position',[70,125,100,20],...
    'Callback',@joint6slider_Callback); %%
%% XYZ Input
XText = uicontrol('Style','edit','String','X',...
    'Position',[25,75,40,20],...
    'Callback',@Xtext_Callback);
YText = uicontrol('Style','edit','String','Y',...
    'Position',[25,50,40,20],...
    'Callback',@Ytext_Callback);
ZText = uicontrol('Style','edit','String','Z',...
    'Position',[25,25,40,20],...
    'Callback',@Ztext_Callback);%%
%% Push Buttons
ResetButton = uicontrol('Style','pushbutton','String','Reset',...
    'Position',[70,60,75,40],...
    'Callback',@ResetButton_Callback);
StartButton = uicontrol('Style','pushbutton','String','Start',...
    'Position',[70,20,75,40],...
    'Callback',@StartButton_Callback);
%% Change units to normalized so components resize
% automatically.
joint1Text.Units = 'normalized';
joint1Slider.Units = 'normalized';
joint2Text.Units = 'normalized';
joint2Slider.Units = 'normalized';
joint3Text.Units = 'normalized';
joint3Slider.Units = 'normalized';
joint4Text.Units = 'normalized';
joint4Slider.Units = 'normalized';
joint5Text.Units = 'normalized';
joint5Slider.Units = 'normalized';
joint6Text.Units = 'normalized';
joint6Slider.Units = 'normalized';
XText.Units = 'normalized';
YText.Units = 'normalized';
ZText.Units = 'normalized';
XVal = '0';
YVal = '0';
ZVal = '0';
jointval1 = 0;
jointval2 = 0;
jointval3 = 0;
jointval4 = 0;
jointval5 = 0;
jointval6 = 0;
ResetButton.Units = 'normalized';
StartButton.Units = 'normalized';%%

f.Name = 'Simple GUI';
% Move the GUI to the center of the screen.
movegui(f,'center')
% Make the GUI visible.
f.Visible = 'on';
%% Slider & Text Callbacks
    function joint1slider_Callback(source,~)
        jointval1 = source.Value;
        set(joint1Text,'String',num2str(jointval1));
    end

    function joint1text_Callback(source,~)
        str = source.String;
        set(joint1Slider,'Value',str2double(str));
    end

    function joint2slider_Callback(source,~)
        jointval2 = source.Value;
        set(joint2Text,'String',num2str(jointval2));
    end

    function joint2text_Callback(source,~)
        str = source.String;
        set(joint2Slider,'Value',str2double(str));
    end
    function joint3slider_Callback(source,~)
        jointval3 = source.Value;
        set(joint3Text,'String',num2str(jointval3));
    end

    function joint3text_Callback(source,~)
        str = source.String;
        set(joint3Slider,'Value',str2double(str));
    end

    function joint4slider_Callback(source,~)
        jointval4 = source.Value;
        set(joint4Text,'String',num2str(jointval4));
    end

    function joint4text_Callback(source,~)
        str = source.String;
        set(joint4Slider,'Value',str2double(str));
    end
    function joint5slider_Callback(source,~)
        jointval5 = source.Value;
        set(joint5Text,'String',num2str(jointval5));
    end

    function joint5text_Callback(source,~)
        str = source.String;
        set(joint5Slider,'Value',str2double(str));
    end

    function joint6slider_Callback(source,~)
        jointval6 = source.Value;
        set(joint6Text,'String',num2str(jointval6));
    end

    function joint6text_Callback(source,~)
        str = source.String;
        set(joint6Slider,'Value',str2double(str));
    end %%
%% XYZ input Callbacks
    function Xtext_Callback(source,~)
        XVal = source.String;
    end
    function Ytext_Callback(source,~)
        YVal = source.String;
    end
    function Ztext_Callback(source,~)
        ZVal = source.String;
    end
%% Push Button Callbacks
    function ResetButton_Callback(source,~)
        set(joint1Text,'String','Joint 1');
        set(joint2Text,'String','Joint 2');
        set(joint3Text,'String','Joint 3');
        set(joint4Text,'String','Joint 4');
        set(joint5Text,'String','Joint 5');
        set(joint6Text,'String','Joint 6');
        set(joint1Slider,'Value',0);
        set(joint2Slider,'Value',0);
        set(joint3Slider,'Value',0);
        set(joint4Slider,'Value',0);
        set(joint5Slider,'Value',0);
        set(joint6Slider,'Value',0);
        set(XText,'String','X');
        set(YText,'String','Y');
        set(ZText,'String','Z');
        set(f, 'HandleVisibility', 'off');
        close all;
        set(f, 'HandleVisibility', 'on');
        XVal = '0';
        YVal = '0';
        ZVal = '0';
        jointval1 = 0;
        jointval2 = 0;
        jointval3 = 0;
        jointval4 = 0;
        jointval5 = 0;
        jointval6 = 0;
    end
    function StartButton_Callback(~,~)
        xyz = [str2double(XVal),str2double(YVal),str2double(ZVal)];
        j = [jointval1,jointval2,jointval3,jointval4,jointval5,jointval6];
        %subplot(2,2,4);
        figure
        if all([xyz,j] == 0)
            disp('Using random values');
            RRTCaller();
        else
            RRTCaller('xyz',xyz,'j',j);
        end
    end
end