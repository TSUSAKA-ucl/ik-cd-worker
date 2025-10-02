const { Project, ScriptTarget, SyntaxKind } = require("ts-morph");

const project = new Project({
  compilerOptions: {
    allowJs: true,
    target: ScriptTarget.ESNext,
    jsx: 1 // JSX = React
  },
});

const files = project.addSourceFilesAtPaths("src/**/*.{js,jsx}");

for (const file of files) {
  const jsxElements = file.getDescendantsOfKind(SyntaxKind.JsxElement);
  const selfClosingElements = file.getDescendantsOfKind(SyntaxKind.JsxSelfClosingElement);
  const jsxExpressions = file.getDescendantsOfKind(SyntaxKind.JsxExpression);

  const allJsxNodes = [...jsxElements, ...selfClosingElements, ...jsxExpressions];

  for (const node of allJsxNodes) {
    const line = node.getStartLineNumber();
    const snippet = node.getText().replace(/\s+/g, ' ').slice(0, 80);
    console.log(`${file.getFilePath()} :: line ${line} :: ${snippet}...`);
  }
}
