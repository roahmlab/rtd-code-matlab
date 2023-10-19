function pZ_out = transpose(pZ)
import armour.pz_roahm.*
% Lazy matrix transpose operation so we don't have to keep saving the
% _t versions of everything

C_t = pZ.C.';
G_t = permute(pZ.G, [2, 1, 3]);
Grest_t = permute(pZ.Grest, [2, 1, 3]);
pZ_out = matPolyZonotope_ROAHM(C_t, G_t, Grest_t, pZ.expMat, pZ.id);

end