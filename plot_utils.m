1;

%==============================================================================
%:::::::: PLOT RESULTS ::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%==============================================================================

% Plot Landmarks and Poses before and after optimization
function fig1 = plotSLAM(XR_truth, XR_guess, XR, XL_truth, XL_guess, XL)
        figure(1,'position',[100,100,1200,800]);
        fig1 = figure(1);
        true_color = [0,.7,0];
        guess_color = [.5,0,.5];
        opt_color = [0,0,.8];
        linewidth = 2;

        subplot(2,2,1);
        ('Landmark: initial guess');
        plot(XL_truth(1,:),XL_truth(2,:),'x','color',true_color,'linewidth',linewidth);
        hold on; grid on;
        plot(XL_guess(1,:),XL_guess(2,:),'o','color',guess_color,'linewidth',linewidth);
        legend('True Landmark', 'Guess Landmark');

        subplot(2,2,2);
        title('Landmark: optimized');
        plot(XL_truth(1,:),XL_truth(2,:),'x','color',true_color,'linewidth',linewidth);
        hold on; grid on;
        plot(XL(1,:),XL(2,:),'o','color',opt_color,'linewidth',linewidth);
        legend('True Landmark', 'Optimized Landmark');

        subplot(2,2,3);
        title('Poses: initial guess');
        plot(squeeze(XR_truth(1,3,:)),squeeze(XR_truth(2,3,:)),'x-','color',true_color,'linewidth',linewidth);
        hold on; grid on;
        plot(squeeze(XR_guess(1,3,:)),squeeze(XR_guess(2,3,:)),'o-','color',guess_color,'linewidth',linewidth);
        legend('True Poses', 'Guess Poses');

        subplot(2,2,4);
        title('Poses: optimized');
        plot(squeeze(XR_truth(1,3,:)),squeeze(XR_truth(2,3,:)),'x-','color',true_color,'linewidth',linewidth);
        hold on; grid on;
        plot(squeeze(XR(1,3,:)),squeeze(XR(2,3,:)),'o-','color',opt_color,'linewidth',linewidth);
        legend('True Poses', 'Optimized Poses');
        
        figure(1,'position',[100,100,1200,800]);
endfunction
%==============================================================================

% Plot Chi evolution over iterations
function fig2 = plotChi(chi_r, chi_l)
        figure(2,'position',[1300,100,600,800]);
        fig2 = figure(2);
        opt_color = [0,0,.7];
        linewidth = 2;

        subplot(3,1,1);
        title('Chi Poses');
        plot(chi_r, '-', 'color', opt_color, 'linewidth', linewidth);
        grid on;
        legend('Chi Poses');
        xlabel('Gauss-Newton iterations');

        subplot(3,1,2);
        title('Chi Landmarks');
        plot(chi_l, '-', 'color', opt_color, 'linewidth', linewidth);
        grid on;
        legend('Chi Landmarks');
        xlabel('Gauss-Newton iterations');
        
        subplot(3,1,3);
        title('Chi');
        plot(chi_r + chi_l, '-', 'color', opt_color, 'linewidth', linewidth);
        grid on;
        legend('Chi');
        xlabel('Gauss-Newton iterations');

        figure(2,'position',[1300,100,600,800]);
endfunction
%==============================================================================

% Plot the structure of the H matrix
function fig3 = plotH(H)
        fig3 = figure(3);
        H_ = H./H;               
        H_(isnan(H_)) = 0;
        imagesc(H_);
        title('H matrix');
endfunction