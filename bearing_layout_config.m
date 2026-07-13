function layout = bearing_layout_config()
%BEARING_LAYOUT_CONFIG Single source for the bearing arrangement.
layout.bearing1.id = 1;
layout.bearing1.type = 'ball';
layout.bearing1.rotor_node = 2;
layout.bearing1.case_node = 3;
layout.bearing1.short_label = 'bearing1 ball';
layout.bearing1.position_label = 'front ball bearing';
layout.bearing1.full_label = 'bearing1 = front angular contact ball bearing';
layout.bearing1.name = 'front high-speed angular contact ball bearing';
layout.bearing1.name_cn = '前支承球轴承';

layout.bearing2.id = 2;
layout.bearing2.type = 'roller';
layout.bearing2.rotor_node = 10;
layout.bearing2.case_node = 10;
layout.bearing2.short_label = 'bearing2 roller';
layout.bearing2.position_label = 'rear roller bearing';
layout.bearing2.full_label = 'bearing2 = rear cylindrical roller bearing';
layout.bearing2.name = 'rear high-speed cylindrical roller bearing';
layout.bearing2.name_cn = '后支承滚子轴承';
end
