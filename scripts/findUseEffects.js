// findUseEffects.js
const { Project, ScriptTarget, SyntaxKind } = require("ts-morph");

const project = new Project({
  // useInMemoryFileSystem: true,
  useInMemoryFileSystem: false,
  compilerOptions: {
    allowJs: true,
    target: ScriptTarget.ESNext,
  },
});

const files = project.addSourceFilesAtPaths("src/**/*.{js,jsx}");
// const filePaths = ['src/app/home.js'];
// const files = project.addSourceFilesAtPaths(filePaths);
// console.log(process.cwd());
// console.log(files.length);
for (const file of files) {
  const effects = file.getDescendantsOfKind(SyntaxKind.CallExpression)
    .filter(call => call.getExpression().getText() === 'React.useEffect');
  for (const effect of effects) {
    const args = effect.getArguments();
    const dep = args[1]?.getText() || '[no deps]';
    console.log(`${file.getFilePath()} :: ${effect.getStartLineNumber()} :: ${dep}`);
  }
}
