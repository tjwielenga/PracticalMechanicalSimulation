# Project Goals and Distribution Plan

Status: agreed direction; project summary and public release take priority

Recorded: September 16, 2026

## Purpose

The primary goal is to make Practical Mechanical Simulation widely available
and useful. No single form of publication serves every audience, so the program
should be presented through several complementary routes rather than choosing
between a Julia package, a paper, a website, or an open repository.

This document records that direction so it can be resumed without requiring
the methods paper to be finished first.

## Audiences

The project should serve:

- engineers and analysts who want to build and simulate mechanisms;
- researchers who want to understand and evaluate the methods;
- developers who want to inspect, reuse, or extend the implementation;
- electronic intelligences that help people find, understand, run, and modify
  the program; and
- the author, who needs a durable summary of the project's terminology,
  capabilities, decisions, and current state.

Electronic intelligences are now an important audience and intermediary. The
project should nevertheless remain understandable to people. The same qualities
help both: explicit terminology, plain-text documentation, defined notation,
small executable examples, clear repository structure, and tests that establish
expected behavior.

The documentation should not be tailored to one particular electronic
intelligence or interface. Markdown, TOML, Lua, Julia source, equations, and
executable tests are durable forms that can be read by people and future tools.

## Complementary publication routes

| Artifact | Main purpose |
|---|---|
| Julia package | Installation, reuse, and discovery by Julia users |
| Public repository | Source, tests, examples, issues, and collaboration |
| Methods paper | Explain and validate the Sparse Fully Consistent Modeling Method |
| Foundation website | Provide a permanent and understandable public entry point |
| SimpView | Let users inspect models and results without understanding the solver |
| Project summary | Preserve the complete project map for people and electronic intelligences |

These artifacts have different jobs:

- The paper explains the problem, the method, comparisons with other
  formulations, and the evidence. It is not the user manual.
- Package documentation explains how to install the program, construct models,
  run analyses, and interpret results.
- The repository provides the executable evidence, source code, examples, and
  detailed technical documentation.
- The Foundation website explains what the project is and directs visitors to
  the package, repository, paper, documentation, examples, and viewer.
- SimpView provides an accessible way to inspect input models, failed analyses,
  successful simulations, static solutions, and modes.

## Documentation principles

The project should favor:

- plain-text Markdown for working documentation;
- stable and clearly defined terminology;
- equations accompanied by definitions of every symbol;
- small, self-contained, executable examples;
- explicit TOML and Lua model interfaces;
- clear links among concepts, models, source files, and tests;
- recorded design decisions and known limitations; and
- reproducible benchmarks that identify the model, code version, tolerances,
  computer, and measurement procedure.

Important knowledge should not exist only in old conversations, binary files,
videos, or undocumented source conventions.

The current methods-paper draft is
[`paper/sparseFullyConsistentMethod.md`](paper/sparseFullyConsistentMethod.md).
Markdown is its working source. PDF and Word files are review and submission
artifacts generated at appropriate checkpoints. The draft is safely parked
while the project summary and public-release work proceed. Useful results,
references, and explanations can continue to be recorded without making
completion of the paper a condition for releasing the program.

## Planned project summary

A top-level `PROJECT_SUMMARY.md` should become the living map of the project. It
should not be a long chronological history. It should let the author, a new
contributor, or an electronic intelligence recover the state of the work
quickly and then follow links to detailed documents.

The summary should cover:

1. Purpose and goals
2. The Sparse Fully Consistent Modeling Method
3. Current planar and spatial capabilities
4. Supported analysis types
5. Modeling elements
6. TOML and Lua model construction
7. Integration, initialization, static, and modal methods
8. Saved results and SimpView
9. Repository architecture
10. Important examples and benchmarks
11. Commands for common workflows
12. Verification status
13. Known limitations
14. Publication and distribution plans
15. Current priorities and deferred work
16. A glossary of project terminology

The summary should link to the detailed manuals rather than duplicate them. A
short current-state section near its beginning should be updated after major
checkpoints.

## Recommended sequence

1. Create `PROJECT_SUMMARY.md` while the development history and decisions are
   still available.
2. Establish a clean public-release version of the repository.
3. Improve installation and first-run instructions, examples, tests, licensing,
   version information, and reproducible verification.
4. Settle the public package name and register it in Julia's package system.
5. Add a Foundation website page that links the program, documentation,
   repository, examples, and SimpView.
6. Let people use the program and incorporate what is learned from that use.
7. Finish the methods paper using the released program, accumulated
   documentation, and reproducible benchmarks.

This sequence provides several useful stopping points. The program can be
found, installed, understood, and extended even if the paper takes longer to
complete. Experience with the released program should also improve the paper's
terminology, examples, and evidence.

## Return point

When this work resumes, the next proposed task is to draft
`PROJECT_SUMMARY.md`, followed by a public-release audit. The paper remains an
important later publication step, but it is not a gate for continued
development or release. The summary should become the central map that keeps
the program, documentation, publication, and distribution work consistent.
