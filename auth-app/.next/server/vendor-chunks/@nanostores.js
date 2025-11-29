"use strict";
/*
 * ATTENTION: An "eval-source-map" devtool has been used.
 * This devtool is neither made for production nor for readable output files.
 * It uses "eval()" calls to create a separate source file with attached SourceMaps in the browser devtools.
 * If you are trying to read the output file, select a different devtool (https://webpack.js.org/configuration/devtool/)
 * or disable the default devtool with "devtool: false".
 * If you are looking for production-ready output files, see mode: "production" (https://webpack.js.org/configuration/mode/).
 */
exports.id = "vendor-chunks/@nanostores";
exports.ids = ["vendor-chunks/@nanostores"];
exports.modules = {

/***/ "(ssr)/./node_modules/@nanostores/react/index.js":
/*!*************************************************!*\
  !*** ./node_modules/@nanostores/react/index.js ***!
  \*************************************************/
/***/ ((__unused_webpack___webpack_module__, __webpack_exports__, __webpack_require__) => {

eval("__webpack_require__.r(__webpack_exports__);\n/* harmony export */ __webpack_require__.d(__webpack_exports__, {\n/* harmony export */   useStore: () => (/* binding */ useStore)\n/* harmony export */ });\n/* harmony import */ var nanostores__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! nanostores */ \"(ssr)/./node_modules/nanostores/listen-keys/index.js\");\n/* harmony import */ var react__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! react */ \"(ssr)/./node_modules/next/dist/server/future/route-modules/app-page/vendored/ssr/react.js\");\n\n\n\nfunction useStore(store, opts = {}) {\n  let subscribe = (0,react__WEBPACK_IMPORTED_MODULE_0__.useCallback)(\n    onChange =>\n      opts.keys\n        ? (0,nanostores__WEBPACK_IMPORTED_MODULE_1__.listenKeys)(store, opts.keys, onChange)\n        : store.listen(onChange),\n    [opts.keys, store]\n  )\n\n  let get = store.get.bind(store)\n\n  return (0,react__WEBPACK_IMPORTED_MODULE_0__.useSyncExternalStore)(subscribe, get, get)\n}\n//# sourceURL=[module]\n//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJmaWxlIjoiKHNzcikvLi9ub2RlX21vZHVsZXMvQG5hbm9zdG9yZXMvcmVhY3QvaW5kZXguanMiLCJtYXBwaW5ncyI6Ijs7Ozs7O0FBQXVDO0FBQ2tCOztBQUVsRCxrQ0FBa0M7QUFDekMsa0JBQWtCLGtEQUFXO0FBQzdCO0FBQ0E7QUFDQSxVQUFVLHNEQUFVO0FBQ3BCO0FBQ0E7QUFDQTs7QUFFQTs7QUFFQSxTQUFTLDJEQUFvQjtBQUM3QiIsInNvdXJjZXMiOlsid2VicGFjazovL2F1dGgtYXBwLy4vbm9kZV9tb2R1bGVzL0BuYW5vc3RvcmVzL3JlYWN0L2luZGV4LmpzP2UxMjMiXSwic291cmNlc0NvbnRlbnQiOlsiaW1wb3J0IHsgbGlzdGVuS2V5cyB9IGZyb20gJ25hbm9zdG9yZXMnXG5pbXBvcnQgeyB1c2VDYWxsYmFjaywgdXNlU3luY0V4dGVybmFsU3RvcmUgfSBmcm9tICdyZWFjdCdcblxuZXhwb3J0IGZ1bmN0aW9uIHVzZVN0b3JlKHN0b3JlLCBvcHRzID0ge30pIHtcbiAgbGV0IHN1YnNjcmliZSA9IHVzZUNhbGxiYWNrKFxuICAgIG9uQ2hhbmdlID0+XG4gICAgICBvcHRzLmtleXNcbiAgICAgICAgPyBsaXN0ZW5LZXlzKHN0b3JlLCBvcHRzLmtleXMsIG9uQ2hhbmdlKVxuICAgICAgICA6IHN0b3JlLmxpc3RlbihvbkNoYW5nZSksXG4gICAgW29wdHMua2V5cywgc3RvcmVdXG4gIClcblxuICBsZXQgZ2V0ID0gc3RvcmUuZ2V0LmJpbmQoc3RvcmUpXG5cbiAgcmV0dXJuIHVzZVN5bmNFeHRlcm5hbFN0b3JlKHN1YnNjcmliZSwgZ2V0LCBnZXQpXG59XG4iXSwibmFtZXMiOltdLCJzb3VyY2VSb290IjoiIn0=\n//# sourceURL=webpack-internal:///(ssr)/./node_modules/@nanostores/react/index.js\n");

/***/ })

};
;