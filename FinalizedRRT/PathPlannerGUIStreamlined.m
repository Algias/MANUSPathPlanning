function PathPlannerGUIStreamlined
f = figure('Visible','off','Position',[360,500,700,300]);
t = uitable(f,'Data',zeros(25,6),'Position',[175 10 500 270]);
t.Units = 'normalized';
%t.FontUnits = 'normalized';
t.ColumnWidth = 'auto';
%% Prevents the box from scaling smaller than its initialization
figszfun = @(~,~) set(f, 'position', max([0 0 400 285], f.Position));
f.SizeChangedFcn = figszfun; %%
%% Starting Joints Text and Sliders
for icreate = 1:6
    
    jointText(icreate) = uicontrol('Style','edit','String',['Joint',' ',int2str(icreate)],...
        'Position',[25,250 - ((icreate-1)*25),40,20],...
        'Callback',@jointtext_Callback);
    
    jointSlider(icreate) = uicontrol('Style','slider','String',['Joint',' ',int2str(icreate)],...
        'Position',[70,250 - ((icreate-1)*25),100,20],...
        'Callback',@jointslider_Callback);
    
    jointText(icreate).Units = 'normalized';
    jointSlider(icreate).Units = 'normalized';
    %jointText(icreate).FontUnits = 'normalized';
    
    jointSlider(icreate).Tag = int2str(icreate);
    jointText(icreate).Tag = int2str(icreate);
    jointSlider(icreate).Min = -pi;
    jointSlider(icreate).Max = pi;
    jointval(icreate) = 0;
end
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
XText.Units = 'normalized';
YText.Units = 'normalized';
ZText.Units = 'normalized';
XVal = '0';
YVal = '0';
ZVal = '0';
ResetButton.Units = 'normalized';
StartButton.Units = 'normalized';%%

f.Name = 'Simple GUI';
% Move the GUI to the center of the screen.
movegui(f,'center')
% Make the GUI visible.
f.Visible = 'on';
%% Slider & Text Callbacks
    function jointslider_Callback(source,~)
        %get(source)
        tag = str2double(source.Tag);
        jointval(tag) = source.Value;
        set(jointText(tag),'String',num2str(jointval(tag)));
        
    end

    function jointtext_Callback(source,~)
        tag = str2double(source.Tag);
        str = str2double(source.String);
        set(jointSlider(tag),'Value',str);
        jointval(tag) = str;
    end
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
    function ResetButton_Callback(~,~)
        for ireset = 1:6
            set(jointText(ireset),'String',['Joint',' ',int2str(ireset)]);
            set(jointSlider(ireset),'Value',0);
            t.Data = zeros(25,6);

        end
        set(XText,'String','X');
        set(YText,'String','Y');
        set(ZText,'String','Z');
        set(f, 'HandleVisibility', 'off');
        close all;
        set(f, 'HandleVisibility', 'on');
        XVal = '0';
        YVal = '0';
        ZVal = '0';
        jointval = zeros(1,6);
    end
    function StartButton_Callback(~,~)
        xyz = [str2double(XVal),str2double(YVal),str2double(ZVal)];
        j = [jointval(1),jointval(2),jointval(3),jointval(4),jointval(5),jointval(6)];
        %subplot(2,2,4);
        figure
        if all([xyz,j] == 0)
            disp('Using random values');
           [returnpath,~] = RRTCaller();
        else
           [returnpath,~] = RRTCaller('xyz',xyz,'j',j);
        end
        t.Data = returnpath;
    end
end