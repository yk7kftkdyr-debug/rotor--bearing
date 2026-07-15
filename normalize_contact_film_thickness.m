function film = normalize_contact_film_thickness(value, shape)
%NORMALIZE_CONTACT_FILM_THICKNESS Enforce the local raw-contact film grid.

if isempty(value) || (isscalar(value) && value == 0)
    film = zeros(shape);
    return;
end
if isscalar(value)
    film = repmat(value, shape);
    return;
end
if ~isequal(size(value), shape)
    error('raw_contact:FilmThicknessShape', ...
        'film_thickness must have size %s.', mat2str(shape));
end
film = value;
end
