# Commits

!!! note
    This Standard is adopted from [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/#specification).

## Motivation

A commit is the basic form of contribution towards a repository. It should describe your changes and additions and be easily readable by other contributors and reviewers.

Using conventional commits can significantly improve collaboration and code management. By adhering to a standardized commit message format we enhance readability and maintainability, making it easier for team members to understand the purpose and context of each change. Consistent commit messages streamline code reviews, simplify tracking changes over time, and facilitate automated processes like triggering CI/CD pipelines. 

## Structure

Conventional commits propose the following overall structure for commits.

```
<type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

!!! note
    In this case optional actually means: If you have the option to leave the information out, leave it out. Only use this if you really need it.

### Types

<table>
  <tr>
    <th>Type</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>feat</td>
    <td>This commit introduces a new feature to the code base.</td>
  </tr>
  <tr>
    <td>fix</td>
    <td>Introducing a fix to a bug or doing minor adjustments.</td>
  </tr>
  <tr>
    <td>build</td>
    <td>Anything related to building the Project or Packages, e.g. CMakeText.txt, ...</td>
  </tr>
  <tr>
    <td>ci</td>
    <td>Changing something on the continuous integration pipeline.</td>
  </tr>
  <tr>
    <td>style</td>
    <td>Anything related to code styling, linting etc.</td>
  </tr>
  <tr>
    <td>refactor</td>
    <td>When you change already working and committed code to let it blend into the code base</td>
  </tr>
  <tr>
    <td>test</td>
    <td>Writing and changing any tests have to be marked by this commit type.</td>
  </tr>
</table>

### Scope

If you want to specify at what scope you added a feature or fix, you can do so by appending the scope to the type in parentheses.

```
feat(lang): add Polish language
```

### Breaking Changes

Every time a major change is done, or a milestone has been hit, it has to be indicated in the commit message. There are two ways of doing that. Either append a ```!``` to the type or add ```BREAKING CHANGE``` as a footer to the message.

```
chore!: drop support for Node 6

BREAKING CHANGE: use JavaScript features not available in Node 6.
```

```
feat(api)!: send an email to the customer when a product is shipped
```