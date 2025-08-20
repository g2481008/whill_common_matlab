function [] = waitPressEnterkey(Stopper)
    waitTime = 1.e-3;
    while Stopper.UserData == -1
        pause(waitTime);
    end
end