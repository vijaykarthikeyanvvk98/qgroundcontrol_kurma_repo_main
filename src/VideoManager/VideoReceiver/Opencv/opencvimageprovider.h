#ifndef OPENCVIMAGEPROVIDER_H
#define OPENCVIMAGEPROVIDER_H

#include <QImage>
#include <QObject>
#include <QQuickImageProvider>

class OpencvImageProvider : public QQuickImageProvider
{
    Q_OBJECT
   public:
    OpencvImageProvider(QObject *parent = nullptr);

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;


   public slots:
    void updateImage(const QImage &image);

   signals:
    void imageChanged();
    void null_image_changed();

   private:
    QImage image;
};

#endif // OPENCVIMAGEPROVIDER_H
