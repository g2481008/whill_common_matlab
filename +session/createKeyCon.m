function fig = createKeyCon()
        width = 0.6;
        height = 0.2;
        x = 0.5-width/2;
        y = 0.5-height/2;
        fig = uifigure('WindowStyle','alwaysontop');
        set(fig,'Name','Key Control', ...
            'KeyPressFcn',@checkSignal, 'UserData',-1);
        uicontrol(fig,'Style','text', ...
            'Units','normalized', ...
            'String',{'Press Enterkey to execute loop','Stop: ESC'}, ...
            'Position', [x,y,width,height], ...
            'FontSize',15);

        function checkSignal(src, event)
            if src.UserData == -1 
                if strcmp(event.Key,'return')
                    % Start when Enter key is pressed
                    disp('Start system');
                    src.UserData = true;
                end
            else
                if strcmp(event.Key, 'escape')
                    % Stop when any key is pressed without Enter
                    disp('Stop system');
                    src.UserData = false;
                end
            end
        end
end