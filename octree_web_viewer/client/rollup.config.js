import commonjs from 'rollup-plugin-commonjs';
import nodeResolve from 'rollup-plugin-node-resolve';

export default {
  input: 'build/main.js',
  output: {
    format: 'iife',
    file: '../../target/app_bundle.js',
    sourcemap: true,
  },
  plugins: [
    commonjs({
      include: 'node_modules/**',
    }),
    nodeResolve({
      browser: true,
      preferBuiltins: false,
    }),
  ],
};

