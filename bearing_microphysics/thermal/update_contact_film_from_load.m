function [film, state] = update_contact_film_from_load(Q, contact_geometry, eta, alpha_p)
%UPDATE_CONTACT_FILM_FROM_LOAD Reduced local Hamrock-Dowson film mapping.
required = {'bearing_type','R_effective','contact_length','E_star', ...
    'entrainment_velocity','reference_viscosity','film_exponent','load_exponent'};
if ~isstruct(contact_geometry) || ~all(isfield(contact_geometry, required)), error('contact_geometry is incomplete.'); end
positive = {'R_effective','contact_length','E_star','entrainment_velocity','reference_viscosity','film_exponent'};
for k=1:numel(positive), validateattributes(contact_geometry.(positive{k}), {'numeric'}, {'scalar','positive','finite'}); end
validateattributes(contact_geometry.load_exponent, {'numeric'}, {'scalar','real','finite'});
validateattributes(eta, {'numeric'}, {'scalar','positive','finite'});
validateattributes(alpha_p, {'numeric'}, {'scalar','positive','finite'});
if ~isnumeric(Q) || any(~isfinite(Q(:))) || any(Q(:)<0), error('Q must be finite and nonnegative.'); end
loaded_mask = Q > 0; h_m = nan(size(Q));
if any(loaded_mask, 'all')
    Re = contact_geometry.R_effective; E_star = contact_geometry.E_star; U = contact_geometry.entrainment_velocity;
    prefactor = 1.6*Re*(eta*U/(E_star*Re))^contact_geometry.film_exponent*(alpha_p*E_star)^0.10;
    switch lower(contact_geometry.bearing_type)
        case 'ball', load_term = Q(loaded_mask)/(E_star*Re^2);
        case 'roller', load_term = Q(loaded_mask)/(E_star*Re*contact_geometry.contact_length);
        otherwise, error('Unsupported bearing type: %s.', contact_geometry.bearing_type);
    end
    h_m(loaded_mask) = prefactor*load_term.^contact_geometry.load_exponent;
end
film = struct('h_m', h_m, 'loaded_mask', loaded_mask, 'hmin_m', min_loaded(h_m, loaded_mask));
state = struct('formula', 'reduced_local_hamrock_dowson', 'loaded_count', nnz(loaded_mask));
end

function hmin_m = min_loaded(h_m, loaded_mask)
if any(loaded_mask, 'all'), hmin_m = min(h_m(loaded_mask)); else, hmin_m = NaN; end
end
