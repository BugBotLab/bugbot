// Sets up legacy OpenCV Module and preloads the cascade in WASM FS.
// Requires <script src="opencv.js"></script> to be included before this module.
if (!window.Module) window.Module = {};
Object.assign(window.Module, {
  wasmBinaryFile: 'opencv_js.wasm',
  locateFile: p => (p.endsWith('.wasm') ? 'opencv_js.wasm' : p),
  preRun: [function(){
    try {
      Module.FS_createPreloadedFile('/', 'haarcascade_frontalface_default.xml',
                                    'haarcascade_frontalface_default.xml', true, false);
    } catch {} // ignore if not present
  }],
  _main(){ /* cv ready; individual panels can check window.cv */ }
});
