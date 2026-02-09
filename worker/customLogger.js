// worker.js の一番最初
const noop = () => {}

const customLogger = {
  debug: noop,
  log:  noop,
  warn:  noop,
  error: noop,
}
export {customLogger};
