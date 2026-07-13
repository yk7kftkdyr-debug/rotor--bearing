function params = enforce_bearing_layout(params)
%ENFORCE_BEARING_LAYOUT Keep the model as front ball + rear roller.
layout = bearing_layout_config();
params.bearing(1).type = layout.bearing1.type;
params.bearing(1).rotor_node = layout.bearing1.rotor_node;
params.bearing(1).case_node = layout.bearing1.case_node;
params.bearing(1).name = layout.bearing1.name;
params.bearing(2).type = layout.bearing2.type;
params.bearing(2).rotor_node = layout.bearing2.rotor_node;
params.bearing(2).case_node = layout.bearing2.case_node;
params.bearing(2).name = layout.bearing2.name;
end
