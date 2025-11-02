%%
%% 
%openExample('aeroblks_quad/QuadcopterProjectExample');  % downloads example (does not overwrite old copies)
openProject('asbQuadcopter');                            % opens the Simulink project and loads needed vars
open_system('asbQuadcopter');                            % top model
%%
% Duplicate the top model into your workspace so the original stays pristine
orig = 'asbQuadcopter';
work = 'asbQuadcopter_RL';
save_system(orig, [work '.slx']);
open_system(work);

%%
open_system('asbQuadcopter_RL');
