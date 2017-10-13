% Doing compression with pnmtopng is faster than use imwrite to save png
function ppmtopng_and_remove_ppm_nowait(ppmfilepath, pngfilepath)
  system(sprintf('pnmtopng -compression 1 %s > %s && rm %s &', ppmfilepath, pngfilepath, ppmfilepath));
end
