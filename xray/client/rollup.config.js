import commonjs from 'rollup-plugin-commonjs';
import nodeResolve from 'rollup-plugin-node-resolve';
import typescript from 'rollup-plugin-typescript';

export default {
  input: 'main.ts',
  output: {
    format: 'iife',
    file: '../../target/xray_app_bundle.js',
    sourcemap: true
  },
  plugins: [
    commonjs({
      include: 'node_modules/**'
    }),
    nodeResolve({
      browser: true,
      preferBuiltins: false
    }),
    typescript()
  ]
};
