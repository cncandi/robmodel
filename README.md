# robmodel

KR8 RobSimul v37 data.

## Deployment

Der Live-Upload läuft wie bei RobSimul über GitHub Actions: Jeder Push auf `main` und jeder manuelle `workflow_dispatch` startet den Workflow `.github/workflows/deploy-ftp.yml`.

Die FTP-Zugangsdaten werden nicht im Repository gespeichert. Lege sie als Repository- oder Organization-Secrets in GitHub Actions ab. Der Workflow akzeptiert beide Namensschemata:

- Server: `FTP_SERVER` oder `FTP_HOST`
- Benutzer: `FTP_USERNAME` oder `FTP_USER`
- Passwort: `FTP_PASSWORD` oder `FTP_PASS`
- Zielordner: `FTP_SERVER_DIR` oder `FTP_TARGET_DIR`

Zum Deployen reicht danach ein Push nach `main`; der Workflow lädt den Repository-Inhalt automatisch in den konfigurierten FTP-Zielordner hoch.
