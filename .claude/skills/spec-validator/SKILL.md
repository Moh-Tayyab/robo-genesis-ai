# SpecValidator Skill

## Description
Validates implementations against specifications, requirements, and best practices.

## Parameters
- `implementation`: The code/feature to validate
- `requirements`: Specifications to validate against
- `validation_type`: Type of validation (security, performance, architecture, etc.)
- `standards`: Standards to check against (best practices, security, etc.)

## Usage Examples
```
skill:spec-validator
validation_type: security
standards: owasp, best-practices
requirements: |
  Authentication system must use secure tokens and rate limiting
implementation: |
  // Better-Auth configuration code...
```

## Output
- Detailed validation report
- Issues found with severity levels
- Recommendations for improvements
- Compliance status against requirements